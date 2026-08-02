#!/usr/bin/env python3
"""
Generic OBD-II diagnostic trouble code (DTC) reader over a WiCAN slcan/TCP
connection. Vehicle-agnostic — standard SAE J1979 PIDs, nothing Hyundai/
E-GMP specific. Point --host at any WiCAN dongle plugged into any OBD-II
port (petrol, diesel, EV) and it will read stored/pending DTCs, VIN, MIL
status and (if any codes are stored) freeze frame data.

This is intentionally separate from bridge.py: that script decodes
manufacturer-specific Mode 22 PIDs for one specific EV platform. This one
only speaks standard OBD-II modes (01, 02, 03, 04, 07, 09) that every
OBD-II-compliant car (post-2001 EU petrol/diesel included) supports.

Usage:
    python3 obd_dtc_reader.py --host 192.168.4.1
    python3 obd_dtc_reader.py --host 192.168.4.1 --json out.json
    python3 obd_dtc_reader.py --host 192.168.4.1 --clear
    python3 obd_dtc_reader.py --host 192.168.4.1 \
        --custom-req-id 0x752 --custom-resp-id 0x652   # e.g. BSI/comfort ECU

Notes on non-engine modules (BSI, ABS, airbag, etc.):
Standard Mode 03/07 only reaches the emissions-relevant ECU (usually the
engine/PCM) via the broadcast functional ID 0x7DF. Modules like Peugeot's
BSI live on manufacturer-specific diagnostic CAN IDs that vary by model/
generation and aren't something this script can guess correctly for you —
you'd need to find the right req/resp arbitration IDs first (vendor
documentation, forums, or sniffing the CAN bus while triggering BSI
functions). Once you have them, --custom-req-id/--custom-resp-id will
send a UDS ReadDTCInformation (0x19 0x02) request to that ECU.
"""
import argparse
import json
import socket
import sys
import time

DEFAULT_PORT = 3333
FUNCTIONAL_REQUEST_ID = 0x7DF
ECU_RESP_RANGE = range(0x7E8, 0x7F0)

DTC_LETTERS = ["P", "C", "B", "U"]

# Common, standardized (SAE J2012) generic powertrain codes. Manufacturer-
# specific codes (P1xxx/C1xxx/B1xxx/U1xxx) are NOT included here — those
# vary by make and aren't safe to guess. Unlisted codes just print with no
# description; paste them to your diagnosis session for a proper read.
GENERIC_DTC_DESCRIPTIONS = {
    "P0011": "Camshaft position timing over-advanced (bank 1)",
    "P0016": "Crankshaft/camshaft position correlation (bank 1 sensor A)",
    "P0030": "HO2S heater control circuit (bank 1 sensor 1)",
    "P0100": "Mass or volume air flow circuit malfunction",
    "P0101": "Mass or volume air flow circuit range/performance",
    "P0105": "Manifold absolute pressure circuit malfunction",
    "P0110": "Intake air temperature circuit malfunction",
    "P0115": "Engine coolant temperature circuit malfunction",
    "P0117": "Engine coolant temperature circuit low input",
    "P0118": "Engine coolant temperature circuit high input",
    "P0120": "Throttle/pedal position sensor A circuit malfunction",
    "P0125": "Insufficient coolant temperature for closed loop fuel control",
    "P0128": "Coolant thermostat (below regulating temperature)",
    "P0130": "O2 sensor circuit malfunction (bank 1 sensor 1)",
    "P0133": "O2 sensor circuit slow response (bank 1 sensor 1)",
    "P0135": "O2 sensor heater circuit malfunction (bank 1 sensor 1)",
    "P0171": "System too lean (bank 1)",
    "P0172": "System too rich (bank 1)",
    "P0174": "System too lean (bank 2)",
    "P0175": "System too rich (bank 2)",
    "P0181": "Fuel temperature sensor A circuit range/performance",
    "P0217": "Engine overtemperature condition",
    "P0230": "Fuel pump primary circuit malfunction",
    "P0234": "Turbocharger/supercharger overboost condition",
    "P0235": "Turbocharger boost sensor A circuit malfunction",
    "P0236": "Turbocharger boost sensor A circuit range/performance",
    "P0243": "Turbocharger wastegate solenoid A malfunction",
    "P0261": "Cylinder 1 injector circuit low",
    "P0299": "Turbocharger/supercharger underboost condition",
    "P0300": "Random/multiple cylinder misfire detected",
    "P0301": "Cylinder 1 misfire detected",
    "P0302": "Cylinder 2 misfire detected",
    "P0303": "Cylinder 3 misfire detected",
    "P0304": "Cylinder 4 misfire detected",
    "P0325": "Knock sensor 1 circuit malfunction",
    "P0335": "Crankshaft position sensor A circuit malfunction",
    "P0340": "Camshaft position sensor A circuit malfunction",
    "P0401": "EGR flow insufficient detected",
    "P0402": "EGR flow excessive detected",
    "P0420": "Catalyst system efficiency below threshold (bank 1)",
    "P0430": "Catalyst system efficiency below threshold (bank 2)",
    "P0440": "EVAP emission control system malfunction",
    "P0442": "EVAP system leak detected (small leak)",
    "P0446": "EVAP vent control circuit malfunction",
    "P0455": "EVAP system leak detected (large leak/no purge flow)",
    "P0456": "EVAP system leak detected (very small leak)",
    "P0500": "Vehicle speed sensor malfunction",
    "P0505": "Idle control system malfunction",
    "P0506": "Idle control system RPM lower than expected",
    "P0507": "Idle control system RPM higher than expected",
    "P0562": "System voltage low",
    "P0563": "System voltage high",
    "P0601": "Internal control module memory checksum error",
    "P0606": "ECM/PCM processor fault",
    "P0700": "Transmission control system malfunction (MIL request from TCM)",
    "P0715": "Input/turbine speed sensor circuit malfunction",
    "P0730": "Incorrect gear ratio",
    "U0100": "Lost communication with ECM/PCM",
    "U0101": "Lost communication with TCM",
    "U0121": "Lost communication with ABS control module",
}


def slcan_tx(can_id: int, data: bytes) -> bytes:
    payload = data.ljust(8, b"\x00")
    return f"t{can_id:03X}8{payload.hex().upper()}\r".encode("ascii")


def slcan_rx_parse(line: str):
    if not line or line[0] != "t" or len(line) < 5:
        return None
    cid = int(line[1:4], 16)
    dlc = int(line[4], 16)
    return cid, bytes.fromhex(line[5 : 5 + dlc * 2])


class WiCAN:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.buf = b""

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.sock.settimeout(0.5)
        self.buf = b""
        self.sock.sendall(b"S6\r")
        time.sleep(0.1)
        self._drain()
        self.sock.sendall(b"O\r")
        time.sleep(0.1)
        self._drain()

    def close(self):
        if self.sock:
            try:
                self.sock.sendall(b"C\r")
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _drain(self):
        self.sock.settimeout(0.05)
        try:
            while self.sock.recv(4096):
                pass
        except Exception:
            pass
        self.buf = b""
        self.sock.settimeout(0.5)

    def _next_line(self, deadline_t):
        while b"\r" not in self.buf:
            timeout = deadline_t - time.monotonic()
            if timeout <= 0:
                return None
            try:
                self.sock.settimeout(min(0.3, timeout))
                chunk = self.sock.recv(4096)
                if not chunk:
                    return None
                self.buf += chunk
            except socket.timeout:
                return None
        line, _, self.buf = self.buf.partition(b"\r")
        try:
            return line.decode("ascii")
        except Exception:
            return None

    def query_single(self, tx_id, rx_id, payload, timeout=1.5):
        """Send request, return first matching SINGLE-frame response payload."""
        self._drain()
        self.sock.sendall(slcan_tx(tx_id, bytes([len(payload)]) + payload))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._next_line(deadline)
            if line is None:
                return None
            parsed = slcan_rx_parse(line)
            if parsed is None:
                continue
            cid, data = parsed
            if rx_id != 0 and cid != rx_id:
                continue
            if rx_id == 0 and not (0x7E8 <= cid <= 0x7EF):
                continue
            if data[0] >> 4 != 0x0:
                continue
            length = data[0] & 0x0F
            return bytes(data[1 : 1 + length])
        return None

    def query_functional_multi(self, tx_id, payload, timeout=2.5):
        """Send a functional (broadcast) request, collect responses from
        every ECU that answers (0x7E8-0x7EF), handling both single- and
        multi-frame ISO-TP per responding ECU independently.

        WiCAN-OBD-C3's slcan TX buffer can drop leading consecutive frames
        of a fast multi-frame burst — same limitation documented in
        bridge.py. We assemble best-effort and track which byte offsets
        actually arrived so callers don't trust padding zeros as real data.

        Returns {cid: (assembled_bytes, valid_offsets_set)}.
        """
        self._drain()
        self.sock.sendall(slcan_tx(tx_id, bytes([len(payload)]) + payload))
        deadline = time.monotonic() + timeout
        state = {}
        results = {}
        while time.monotonic() < deadline:
            line = self._next_line(deadline)
            if line is None:
                break
            parsed = slcan_rx_parse(line)
            if parsed is None:
                continue
            cid, data = parsed
            if cid not in ECU_RESP_RANGE:
                continue
            pci = data[0] >> 4
            if pci == 0x0:
                length = data[0] & 0x0F
                results[cid] = (bytes(data[1 : 1 + length]), set(range(length)))
            elif pci == 0x1:
                expected_len = ((data[0] & 0x0F) << 8) | data[1]
                state[cid] = {"ff": data, "expected_len": expected_len, "cfs": {}}
                # Physical response IDs (7E8-7EF) map back to request IDs (7E0-7E7).
                self.sock.sendall(slcan_tx(cid - 8, bytes([0x30, 0x00, 0x0A])))
            elif pci == 0x2:
                st = state.get(cid)
                if st is not None:
                    st["cfs"][data[0] & 0x0F] = data[1:8]

        for cid, st in state.items():
            ff = st["ff"]
            expected_len = st["expected_len"]
            cfs = st["cfs"]
            valid = set(range(6))
            assembled = bytearray(ff[2:8])
            max_cfs = (expected_len - 6 + 6) // 7 + 1
            for seq in range(1, max_cfs + 1):
                if seq not in cfs:
                    continue
                start = 6 + (seq - 1) * 7
                while len(assembled) < start:
                    assembled.append(0)
                cf = cfs[seq]
                end = min(start + 7, expected_len)
                for i in range(end - start):
                    if start + i < len(assembled):
                        assembled[start + i] = cf[i]
                    else:
                        assembled.append(cf[i])
                valid.update(range(start, end))
            results[cid] = (bytes(assembled[:expected_len]), valid)
        return results

    def query_iso_tp(self, tx_id, rx_id, payload, timeout=2.5):
        """Physically-addressed single-ECU query (used for --custom-req-id
        and Mode 04 clear). Returns (assembled_bytes, valid_offsets)."""
        results = self.query_functional_multi_physical(tx_id, rx_id, payload, timeout)
        return results.get(rx_id, (None, set()))

    def query_functional_multi_physical(self, tx_id, rx_id, payload, timeout=2.5):
        # Flow control must go to the ECU's dedicated physical request ID.
        # When tx_id is the broadcast/functional ID (0x7DF), that's rx_id-8
        # (the standard 7E0-7E7 <-> 7E8-7EF mapping) — sending FC back to
        # 0x7DF itself would address every ECU, not the one we're talking to.
        # For an already-physical custom req/resp pair, tx_id IS that ID.
        fc_id = (rx_id - 8) if tx_id == FUNCTIONAL_REQUEST_ID else tx_id
        self._drain()
        self.sock.sendall(slcan_tx(tx_id, bytes([len(payload)]) + payload))
        deadline = time.monotonic() + timeout
        ff = None
        cfs = {}
        expected_len = None
        while time.monotonic() < deadline:
            line = self._next_line(deadline)
            if line is None:
                break
            parsed = slcan_rx_parse(line)
            if parsed is None:
                continue
            cid, data = parsed
            if cid != rx_id:
                continue
            pci = data[0] >> 4
            if pci == 0x0:
                length = data[0] & 0x0F
                return {rx_id: (bytes(data[1 : 1 + length]), set(range(length)))}
            elif pci == 0x1:
                ff = data
                expected_len = ((data[0] & 0x0F) << 8) | data[1]
                self.sock.sendall(slcan_tx(fc_id, bytes([0x30, 0x00, 0x0A])))
            elif pci == 0x2:
                cfs[data[0] & 0x0F] = data[1:8]
        if ff is None:
            return {rx_id: (None, set())}
        valid = set(range(6))
        assembled = bytearray(ff[2:8])
        max_cfs = (expected_len - 6 + 6) // 7 + 1
        for seq in range(1, max_cfs + 1):
            if seq not in cfs:
                continue
            start = 6 + (seq - 1) * 7
            while len(assembled) < start:
                assembled.append(0)
            cf = cfs[seq]
            end = min(start + 7, expected_len)
            for i in range(end - start):
                if start + i < len(assembled):
                    assembled[start + i] = cf[i]
                else:
                    assembled.append(cf[i])
            valid.update(range(start, end))
        return {rx_id: (bytes(assembled[:expected_len]), valid)}


def decode_dtc_pair(hi: int, lo: int):
    if hi == 0 and lo == 0:
        return None
    letter = DTC_LETTERS[(hi >> 6) & 0x3]
    digit1 = (hi >> 4) & 0x3
    digit2 = hi & 0x0F
    return f"{letter}{digit1}{digit2:X}{lo:02X}"


def extract_dtcs(data: bytes, valid: set, expected_sid: int):
    """data[0]=SID, data[1]=count, then 2-byte pairs from offset 2."""
    if not data or len(data) < 2 or 0 not in valid or 1 not in valid:
        return []
    if data[0] != expected_sid:
        return []
    codes = []
    offset = 2
    while offset + 1 < len(data):
        if offset in valid and (offset + 1) in valid:
            code = decode_dtc_pair(data[offset], data[offset + 1])
            if code:
                codes.append(code)
        else:
            codes.append("??? (bytes dropped in transit — rerun or move closer to WiFi)")
        offset += 2
    return codes


def decode_uds_dtcs(data: bytes, valid: set):
    """UDS ReadDTCInformation (0x19 0x02) positive response: 59 02 <availMask>
    then repeating [DTC_hi, DTC_mid, DTC_lo, status] groups (4 bytes each)."""
    if not data or len(data) < 3 or data[0] != 0x59:
        return []
    out = []
    offset = 3
    while offset + 3 < len(data):
        if all((offset + i) in valid for i in range(4)):
            hi, mid, lo, status = data[offset : offset + 4]
            if not (hi == 0 and mid == 0 and lo == 0):
                letter = DTC_LETTERS[(hi >> 6) & 0x3]
                digit1 = (hi >> 4) & 0x3
                digit2 = hi & 0x0F
                code = f"{letter}{digit1}{digit2:X}{mid:02X}"
                out.append((code, lo, status))
        else:
            out.append(("??? (bytes dropped)", None, None))
        offset += 4
    return out


FREEZE_FRAME_PIDS = {
    0x04: ("engine_load_pct", lambda d: round(d[0] * 100 / 255, 1)),
    0x05: ("coolant_temp_c", lambda d: d[0] - 40),
    0x06: ("short_fuel_trim_b1_pct", lambda d: round((d[0] - 128) * 100 / 128, 1)),
    0x07: ("long_fuel_trim_b1_pct", lambda d: round((d[0] - 128) * 100 / 128, 1)),
    0x0B: ("intake_map_kpa", lambda d: d[0]),
    0x0C: ("rpm", lambda d: round(((d[0] << 8) | d[1]) / 4, 0)),
    0x0D: ("speed_kph", lambda d: d[0]),
    0x0F: ("intake_air_temp_c", lambda d: d[0] - 40),
    0x10: ("maf_gps", lambda d: round(((d[0] << 8) | d[1]) / 100, 2)),
    0x11: ("throttle_pct", lambda d: round(d[0] * 100 / 255, 1)),
    0x1F: ("runtime_since_start_s", lambda d: (d[0] << 8) | d[1]),
    0x2F: ("fuel_level_pct", lambda d: round(d[0] * 100 / 255, 1)),
}


def read_freeze_frame(wican: WiCAN):
    out = {}
    for pid, (name, fn) in FREEZE_FRAME_PIDS.items():
        resp = wican.query_single(FUNCTIONAL_REQUEST_ID, 0, bytes([0x02, pid, 0x00]))
        if not resp or len(resp) < 3 or resp[0] != 0x42 or resp[1] != pid:
            continue
        try:
            out[name] = fn(resp[3:])
        except Exception:
            continue
    return out


def read_vin(wican: WiCAN):
    resp = wican.query_functional_multi(FUNCTIONAL_REQUEST_ID, bytes([0x09, 0x02]))
    for cid, (data, valid) in resp.items():
        if not data or len(data) < 20 or data[0] != 0x49 or data[1] != 0x02:
            continue
        if not all(o in valid for o in range(3, 20)):
            continue
        try:
            vin = data[3:20].decode("ascii")
            if len(vin) == 17 and vin.isalnum():
                return vin
        except Exception:
            pass
    return None


def read_mil_status(wican: WiCAN):
    """Mode 01 PID 00 supported PIDs isn't needed; PID 01 gives MIL + DTC count."""
    resp = wican.query_single(FUNCTIONAL_REQUEST_ID, 0, bytes([0x01, 0x01]))
    if not resp or len(resp) < 3 or resp[0] != 0x41 or resp[1] != 0x01:
        return None
    a = resp[2]
    return {"mil_on": bool(a & 0x80), "dtc_count": a & 0x7F}


def read_stored_and_pending(wican: WiCAN):
    stored, pending = {}, {}
    for mode, sid, bucket in [(0x03, 0x43, stored), (0x07, 0x47, pending)]:
        resp = wican.query_functional_multi(FUNCTIONAL_REQUEST_ID, bytes([mode]))
        for cid, (data, valid) in resp.items():
            codes = extract_dtcs(data, valid, sid)
            if codes:
                bucket[f"0x{cid:03X}"] = codes
    return stored, pending


def clear_dtcs(wican: WiCAN):
    result = wican.query_iso_tp(FUNCTIONAL_REQUEST_ID, 0x7E8, bytes([0x04]), timeout=2.0)
    data, _ = result.get(0x7E8, (None, set()))
    if data and len(data) >= 1 and data[0] == 0x44:
        return True
    if data and len(data) >= 2 and data[0] == 0x7F:
        print(f"Clear rejected, negative response code 0x{data[2]:02X}" if len(data) >= 3 else "Clear rejected", file=sys.stderr)
        return False
    return False


def read_custom_ecu_dtcs(wican: WiCAN, req_id: int, resp_id: int):
    result = wican.query_iso_tp(req_id, resp_id, bytes([0x19, 0x02, 0xFF]), timeout=3.0)
    data, valid = result.get(resp_id, (None, set()))
    if data is None:
        return None
    return decode_uds_dtcs(data, valid)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True, help="WiCAN IP address")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--no-freeze-frame", action="store_true", help="skip freeze frame PIDs")
    ap.add_argument("--no-vin", action="store_true", help="skip VIN read")
    ap.add_argument("--clear", action="store_true", help="clear stored DTCs (Mode 04) after confirmation")
    ap.add_argument("--json", metavar="FILE", help="also write results as JSON to FILE")
    ap.add_argument("--custom-req-id", type=lambda x: int(x, 0), help="hex CAN ID to send UDS DTC request to (e.g. BSI), e.g. 0x752")
    ap.add_argument("--custom-resp-id", type=lambda x: int(x, 0), help="hex CAN ID to expect the response on, e.g. 0x652")
    args = ap.parse_args()

    if bool(args.custom_req_id) != bool(args.custom_resp_id):
        ap.error("--custom-req-id and --custom-resp-id must be given together")

    wican = WiCAN(args.host, args.port)
    print(f"Connecting to WiCAN at {args.host}:{args.port} ...")
    wican.connect()
    print("Connected.\n")

    result = {}

    try:
        mil = read_mil_status(wican)
        result["mil_status"] = mil
        if mil:
            print(f"MIL (check engine light): {'ON' if mil['mil_on'] else 'off'}  |  confirmed DTC count: {mil['dtc_count']}")
        else:
            print("Mode 01 PID 01 did not respond — engine ECU may be asleep/not powered.")

        if not args.no_vin:
            vin = read_vin(wican)
            result["vin"] = vin
            print(f"VIN: {vin or '(no response)'}")

        print("\nReading stored (Mode 03) and pending (Mode 07) DTCs...")
        stored, pending = read_stored_and_pending(wican)
        result["stored_dtcs"] = stored
        result["pending_dtcs"] = pending

        if not stored and not pending:
            print("No DTCs reported by any responding ECU.")
        for label, bucket in [("Stored", stored), ("Pending", pending)]:
            for ecu, codes in bucket.items():
                print(f"  [{label}] ECU {ecu}:")
                for code in codes:
                    desc = GENERIC_DTC_DESCRIPTIONS.get(code, "")
                    print(f"    {code}" + (f"  — {desc}" if desc else ""))

        if stored and not args.no_freeze_frame:
            print("\nReading freeze frame (snapshot at moment first stored DTC was set)...")
            ff = read_freeze_frame(wican)
            result["freeze_frame"] = ff
            for k, v in ff.items():
                print(f"  {k}: {v}")
            if not ff:
                print("  (no freeze frame data returned)")

        if args.custom_req_id:
            print(f"\nQuerying custom ECU req=0x{args.custom_req_id:03X} resp=0x{args.custom_resp_id:03X} "
                  f"(UDS ReadDTCInformation)...")
            custom = read_custom_ecu_dtcs(wican, args.custom_req_id, args.custom_resp_id)
            if custom is None:
                print("  No response — wrong CAN ID, module asleep, or it doesn't support UDS 0x19.")
            elif not custom:
                print("  No DTCs reported.")
            else:
                result["custom_ecu_dtcs"] = []
                for code, status_severity_byte, status in custom:
                    print(f"  {code}  raw_status=0x{status:02X}" if status is not None else f"  {code}")
                    result["custom_ecu_dtcs"].append({"code": code, "status": status})

        if args.clear:
            print(f"\nAbout to send Mode 04 (CLEAR ALL DTCs) to the engine ECU.")
            print("This also resets emissions readiness monitors — the car may need a drive cycle before")
            print("it reports 'ready' for inspection/MOT again. Note down the codes above first.")
            confirm = input("Type 'yes' to proceed: ").strip().lower()
            if confirm == "yes":
                ok = clear_dtcs(wican)
                print("Cleared." if ok else "Clear failed or was rejected.")
                result["cleared"] = ok
            else:
                print("Skipped.")

    finally:
        wican.close()

    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nWrote {args.json}")


if __name__ == "__main__":
    main()
