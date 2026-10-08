"""
WiCAN slcan -> MQTT bridge for Hyundai Ioniq 5.

The WiCAN-OBD-C3's slcan TX buffer is too small to deliver the leading
consecutive frames of a multi-frame ISO-TP burst. We can't fix that in
software (the ECU ignores ISO-TP flow control and bursts).

Strategy that works:
1. **Standard OBD-II Mode 01 PID 5B** ("hybrid battery pack remaining life")
   returns SoC% in a SINGLE CAN frame. No ISO-TP, no buffer issue. This is the
   primary live value — polled every POLL_SECONDS (default 15 min).
2. **Mode 22 PID 0101** (BMS basic) — even though we lose the first ~4 CFs,
   the *trailing* CFs reliably contain cumulative_charge_kWh,
   cumulative_discharge_kWh and operating_time_s. These are slow-changing
   lifetime metrics — perfect for trip/charging analytics derived from deltas.

Polling is gated on the 12 V battery voltage the WiCAN reports over HTTP
(/check_status — no CAN traffic). At or above AWAKE_VOLTAGE the car's DC-DC
converter is running (driving or charging), so the 12 V is being charged and
polling can't drain it; below it the car is asleep and no CAN frames are sent.
While awake, fast PIDs are polled every POLL_SECONDS and slow ones (TPMS,
odometer, range) every SLOW_POLL_SECONDS. Pair with the WiCAN's own sleep mode
(~13 V) so the dongle itself drops to <1 mA when the car is parked.

Published topics (under TOPIC_PREFIX):
- soc_pct                            — % from PID 5B
- cumulative_charge_kwh              — lifetime kWh charged into pack
- cumulative_discharge_kwh           — lifetime kWh discharged from pack
- operating_time_s                   — cumulative powered-on seconds
- 12v_battery_v / car_awake          — every presence check while WiCAN is reachable
- online                             — retained 1/0 LWT
"""
import json
import logging
import os
import socket
import sys
import time
import urllib.request

import paho.mqtt.client as mqtt

WICAN_HOST = os.environ.get("WICAN_HOST", "192.168.0.12")
WICAN_PORT = int(os.environ.get("WICAN_PORT", "3333"))
MQTT_HOST = os.environ["MQTT_HOST"]
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USER = os.environ["MQTT_USER"]
MQTT_PASSWORD = os.environ["MQTT_PASSWORD"]
TOPIC_PREFIX = os.environ.get("TOPIC_PREFIX", "ioniq5")
POLL_SECONDS = int(os.environ.get("POLL_SECONDS", "300"))  # fast PIDs, while car awake
SLOW_POLL_SECONDS = int(os.environ.get("SLOW_POLL_SECONDS", "1800"))  # TPMS/odometer/DTE
PRESENCE_CHECK_SECONDS = int(os.environ.get("PRESENCE_CHECK_SECONDS", "60"))
# DC-DC converter running (car on or charging) holds the 12 V at ~13.5-14.8 V; a
# resting battery sits at ~12.4-12.9 V. Only touch the CAN bus above this.
AWAKE_VOLTAGE = float(os.environ.get("AWAKE_VOLTAGE", "13.2"))
STATE_PATH = os.environ.get("STATE_PATH", "/data/state.json")
# Charging is "in progress" if cumulative_charge_kwh moves by more than this
# between consecutive polls. Below this we treat it as noise/regen-blip.
CHARGE_DELTA_THRESHOLD_KWH = 0.05
# Journey is "in progress" if operating_time advanced between polls.
# (Driving even briefly increases operating_time, so any delta counts.)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    stream=sys.stdout,
)
log = logging.getLogger("wican-bridge")


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
        log.info("WiCAN slcan opened (%s:%s)", self.host, self.port)

    def close(self):
        if self.sock:
            try:
                self.sock.sendall(b"C\r")  # close slcan channel before dropping TCP
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
        """Send request, return the first matching SINGLE-frame response payload."""
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
            # Mode 01 response can come from many ECUs (7E8-7EF). Accept any in that range
            # if rx_id is given as 0 (wildcard).
            if rx_id != 0 and cid != rx_id:
                continue
            if cid < 0x7E8 or cid > 0x7EF:
                if rx_id == 0:
                    continue
            pci = data[0] >> 4
            if pci != 0x0:
                continue  # not a single frame — ignore
            length = data[0] & 0x0F
            return bytes(data[1 : 1 + length])
        return None

    def open_extended_session(self, tx_id, rx_id, timeout=0.5):
        """Send UDS DiagnosticSessionControl 10 03 (extended). Returns True
        if positive (50 03) response seen. Some ECUs gate certain PIDs behind
        this session."""
        self._drain()
        self.sock.sendall(slcan_tx(tx_id, bytes.fromhex("021003")))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._next_line(deadline)
            if line is None:
                return False
            parsed = slcan_rx_parse(line)
            if parsed is None:
                continue
            cid, data = parsed
            if cid != rx_id:
                continue
            if data[0] == 0x06 and data[1] == 0x50 and data[2] == 0x03:
                return True
            if data[0] == 0x03 and data[1] == 0x7F:
                return False
        return False

    def query_iso_tp_late_cfs(self, tx_id, rx_id, payload, timeout=2.5):
        """Send request, assemble what we can (leading CFs are typically lost
        on WiCAN-OBD-C3). Returns (best_effort_bytes, expected_len) so the
        caller knows which absolute byte offsets are present.
        """
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
                return bytes(data[1 : 1 + length]), length, set(range(length))
            elif pci == 0x1:
                ff = data
                expected_len = ((data[0] & 0x0F) << 8) | data[1]
                # Fire FC anyway in case it helps something.
                self.sock.sendall(slcan_tx(tx_id, bytes([0x30, 0x00, 0x0A])))
            elif pci == 0x2:
                seq = data[0] & 0x0F
                cfs[seq] = data[1:8]

        if ff is None:
            return None, 0, set()
        # Assemble what we have, tracking which absolute byte offsets are valid.
        # FF carries response bytes 0..5.
        valid = set(range(6))
        assembled = bytearray(ff[2:8])
        # Pad CFs into their slots. CFs are 7 bytes each; CF seq=k carries bytes 6+(k-1)*7..6+k*7-1.
        # The buffer-drop pattern keeps the LAST few CFs, so we walk all possible seqs.
        max_cfs = (expected_len - 6 + 6) // 7 + 1
        for seq in range(1, max_cfs + 1):
            if seq in cfs:
                # offset where CF seq lives:
                start = 6 + (seq - 1) * 7
                # Pad assembled to that offset.
                while len(assembled) < start:
                    assembled.append(0)
                cf = cfs[seq]
                end = min(start + 7, expected_len)
                # Copy with overwrite (in case some bytes were padding).
                for i in range(end - start):
                    if start + i < len(assembled):
                        assembled[start + i] = cf[i]
                    else:
                        assembled.append(cf[i])
                for ofs in range(start, end):
                    valid.add(ofs)
        # Truncate to expected_len.
        return bytes(assembled[:expected_len]), expected_len, valid


def b_to_off(B: int):
    """Map a WiCAN profile 'B<n>' byte index to response[] byte offset.

    The WiCAN profile expressions count every wire byte including ISO-TP PCI
    bytes. FF has 2 PCI bytes then 6 data bytes; each CF has 1 PCI then 7 data.
    Returns None if the requested B-index falls on a PCI byte.
    """
    if B < 2:
        return None
    if B < 8:
        return B - 2  # FF data bytes
    cf = (B - 8) // 8
    pos = (B - 8) % 8
    if pos == 0:
        return None  # CF PCI byte
    return 6 + cf * 7 + (pos - 1)


def _b(resp, B, default=None):
    off = b_to_off(B)
    if off is None or off >= len(resp):
        return default
    return resp[off]


def _s(resp, B, default=None):
    """Signed 8-bit at B."""
    v = _b(resp, B, default)
    if v is None or v is default:
        return default
    return v - 256 if v >= 128 else v


def _u16(resp, hi_B, lo_B, default=None):
    h = _b(resp, hi_B)
    l = _b(resp, lo_B)
    if h is None or l is None:
        return default
    return (h << 8) | l


def decode_pid5b_soc(data: bytes):
    """Mode 01 PID 5B: A * 100/255 = SoC%."""
    if len(data) < 3 or data[0] != 0x41 or data[1] != 0x5B:
        return None
    return round(data[2] * 100.0 / 255.0, 1)


def decode_pid46_ambient(data: bytes):
    """Mode 01 PID 46: ambient air temperature, A - 40 = °C."""
    if len(data) < 3 or data[0] != 0x41 or data[1] != 0x46:
        return None
    return data[2] - 40


def decode_pid0d_speed(data: bytes):
    """Mode 01 PID 0D: vehicle speed, A = km/h."""
    if len(data) < 3 or data[0] != 0x41 or data[1] != 0x0D:
        return None
    return data[2]


def decode_vmcu_dte(resp: bytes, valid: set):
    """VMCU PID 0101 predicted range / DTE.

    Community-reported offset for E-GMP is u16 at bytes 26-27 (km).
    NOT bench-verified on ST71GRZ — see logged raw bytes and recalibrate
    against the dashboard's "miles remaining" reading."""
    if len(resp) < 3 or resp[0] != 0x62 or resp[1] != 0x01 or resp[2] != 0x01:
        return None
    if not all((26 + i) in valid for i in range(2)):
        return None
    raw = int.from_bytes(resp[26:28], "big")
    if raw == 0 or raw > 1000:
        return None
    return raw  # km


def decode_22_01_05_soh(resp: bytes):
    """BMS extended PID 0105: SOH lives at offset 28-29 (u16/10)."""
    if len(resp) < 30 or resp[0] != 0x62 or resp[1] != 0x01 or resp[2] != 0x05:
        return None
    raw = int.from_bytes(resp[28:30], "big")
    return raw / 10.0


def decode_22_01_05_extra(resp: bytes):
    """Extra fields from BMS extended PID 0105 (WiCAN profile expressions)."""
    if len(resp) < 33 or resp[0] != 0x62 or resp[1] != 0x01 or resp[2] != 0x05:
        return {}
    out = {}
    # HV_KWH_R = ((B37<<8) + B38) * 2 — pack remaining energy in Wh.
    v = _u16(resp, 37, 38)
    if v is not None:
        out["hv_kwh_remaining"] = round(v * 2 / 1000.0, 2)
    return out


def decode_22_01_01_live(resp: bytes):
    """Live/flag fields from BMS PID 0101. Many are 0 when parked-but-READY."""
    if len(resp) < 30 or resp[0] != 0x62 or resp[1] != 0x01 or resp[2] != 0x01:
        return {}
    out = {}
    # B15 bit flags
    flags = _b(resp, 15)
    if flags is not None:
        out["charging"] = (flags >> 7) & 1
        out["charging_dc"] = (flags >> 6) & 1
        out["ac_plug"] = (flags >> 5) & 1
        out["ignition_b15"] = (flags >> 2) & 1
        out["hv_relay"] = flags & 1
    # SoC display, cross-check vs PID 5B (B10/2)
    soc_d = _b(resp, 10)
    if soc_d is not None:
        out["soc_display_pct"] = soc_d / 2.0
    # Battery temp extremes (signed °C)
    t_max = _s(resp, 21)
    t_min = _s(resp, 22)
    if t_max is not None:
        out["hv_t_max"] = t_max
    if t_min is not None:
        out["hv_t_min"] = t_min
    # Cell voltage extremes (50 mV units → V)
    cv_max = _b(resp, 31)
    cv_min = _b(resp, 34)
    if cv_max is not None:
        out["hv_cell_v_max"] = round(cv_max / 50.0, 3)
    if cv_min is not None:
        out["hv_cell_v_min"] = round(cv_min / 50.0, 3)
    if cv_max is not None and cv_min is not None:
        out["hv_cell_v_diff_mv"] = round((cv_max - cv_min) * 20, 1)  # 50 mV/unit → 20 mV gap per unit
    # Live pack — only valid when contactors closed (driving/charging)
    hv_v = _u16(resp, 19, 20)
    if hv_v is not None and hv_v > 0:
        out["hv_pack_v"] = round(hv_v / 10.0, 1)
    hv_a_raw_s = _s(resp, 17)
    hv_a_raw_b = _b(resp, 18)
    if hv_a_raw_s is not None and hv_a_raw_b is not None:
        hv_a = (hv_a_raw_s * 256 + hv_a_raw_b) / 10.0
        if hv_v and hv_v > 0:
            out["hv_pack_a"] = round(hv_a, 1)
            out["hv_pack_w"] = round(hv_a * (hv_v / 10.0), 0)
    return out


def decode_vin(resp: bytes):
    """UDS 22 F1 90 → vehicle identification number as ASCII (17 chars)."""
    if len(resp) < 20 or resp[0] != 0x62 or resp[1] != 0xF1 or resp[2] != 0x90:
        return None
    vin_bytes = resp[3:20]  # 17 chars
    try:
        s = vin_bytes.decode("ascii")
        if len(s) == 17 and s.isalnum():
            return s
    except Exception:
        pass
    return None


def decode_odometer(resp: bytes):
    """UDS 22 B0 02 on Hyundai cluster (7C6 -> 7CE).
    Calibrated against ST71GRZ dashboard 2026-05-17: total miles = Int24 BE
    at data[9:12]. The Int24 at data[3:6] is a different counter (maybe
    service-interval distance) and intentionally ignored."""
    if len(resp) < 15 or resp[0] != 0x62 or resp[1] != 0xB0 or resp[2] != 0x02:
        return None
    data = resp[3:]
    return {"odometer_mi": int.from_bytes(data[9:12], "big")}


def decode_22_c0_0b_tpms(resp: bytes):
    """TPMS module PID C00B response: 4× tyre pressures (psi) and temps (°C)."""
    if len(resp) < 30 or resp[0] != 0x62 or resp[1] != 0xC0 or resp[2] != 0x0B:
        return {}
    out = {}
    # Profile says TYRE_P_FL=B10*0.2 etc. Multiply 0x?? by 0.2 for psi.
    mappings = [
        ("tyre_p_fl_psi", 10, 0.2), ("tyre_t_fl_c", 11, 1, -50),
        ("tyre_p_fr_psi", 15, 0.2), ("tyre_t_fr_c", 17, 1, -50),
        ("tyre_p_rl_psi", 21, 0.2), ("tyre_t_rl_c", 22, 1, -50),
        ("tyre_p_rr_psi", 27, 0.2), ("tyre_t_rr_c", 28, 1, -50),
    ]
    for m in mappings:
        name, B, scale, *bias = m
        v = _b(resp, B)
        if v is None:
            continue
        out[name] = round(v * scale + (bias[0] if bias else 0), 1)
    return out


def load_state():
    try:
        with open(STATE_PATH) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as e:
        log.warning("state load failed: %s", e)
        return {}


def save_state(state):
    try:
        os.makedirs(os.path.dirname(STATE_PATH), exist_ok=True)
        with open(STATE_PATH, "w") as f:
            json.dump(state, f, indent=2)
    except Exception as e:
        log.warning("state save failed: %s", e)


def end_journey(state, end, mq):
    """Publish the active journey's summary, using `end` (a `current` dict) as its end."""
    journey = state["journey"]
    duration_s = end["op_time_s"] - journey["start_op_time"]
    kwh_used = end["discharge_kwh"] - journey["start_discharge"]
    kwh_regen = end["charge_kwh"] - journey["start_charge"]
    soc_delta = end["soc"] - journey["start_soc"]
    net_kwh = kwh_used - kwh_regen
    duration_min = duration_s / 60.0 if duration_s > 0 else 0
    summary = {
        "duration_min": round(duration_min, 1),
        "kwh_used": round(kwh_used, 2),
        "kwh_regen": round(kwh_regen, 2),
        "kwh_net": round(net_kwh, 2),
        "soc_delta": round(soc_delta, 1),
        "start_soc": journey["start_soc"],
        "end_soc": end["soc"],
        "start_ts": journey["start_ts"],
        "end_ts": end["ts"],
    }
    start_odo = journey.get("start_odo_mi")
    end_odo = end.get("odometer_mi")
    if start_odo is not None and end_odo is not None and end_odo >= start_odo:
        miles = end_odo - start_odo
        summary["miles"] = miles
        summary["start_odo_mi"] = start_odo
        summary["end_odo_mi"] = end_odo
        if net_kwh > 0:
            summary["mi_per_kwh"] = round(miles / net_kwh, 2)
    for k, v in summary.items():
        mq.publish(f"{TOPIC_PREFIX}/journey/last_{k}", str(v), retain=True)
    mq.publish(f"{TOPIC_PREFIX}/journey/in_progress", "0", retain=True)
    log.info("journey END: %s", summary)
    journey["active"] = False


def end_charge(state, end, mq):
    """Publish the active charge session's summary, using `end` as its end."""
    charge = state["charge"]
    kwh_added = end["charge_kwh"] - charge["start_charge"]
    soc_added = end["soc"] - charge["start_soc"]
    duration_s = end["ts"] - charge["start_ts"]
    summary = {
        "duration_min": round(duration_s / 60.0, 1),
        "kwh_added": round(kwh_added, 2),
        "soc_added": round(soc_added, 1),
        "start_soc": charge["start_soc"],
        "end_soc": end["soc"],
        "avg_kw": round(kwh_added / (duration_s / 3600.0), 2) if duration_s > 0 else 0,
        "start_ts": charge["start_ts"],
        "end_ts": end["ts"],
    }
    for k, v in summary.items():
        mq.publish(f"{TOPIC_PREFIX}/charge/last_{k}", str(v), retain=True)
    mq.publish(f"{TOPIC_PREFIX}/charge/in_progress", "0", retain=True)
    log.info("charge END: %s", summary)
    charge["active"] = False


def end_sessions_on_sleep(state, mq):
    """Car went to sleep (12 V dropped / WiCAN gone): close any open session at the
    last poll, since no further polls will happen until it wakes again."""
    end = state.get("prev")
    if not end:
        return
    if state.get("journey", {}).get("active"):
        end_journey(state, end, mq)
    if state.get("charge", {}).get("active"):
        end_charge(state, end, mq)
    save_state(state)


def update_session_state(state, current, mq, awake_since):
    """Detect journey + charge session edges, publish summaries on transitions.

    `current` keys: ts, soc, op_time_s, discharge_kwh, charge_kwh, odometer_mi,
    charging_flag (BMS B15 bit 7, or None if that byte was lost).
    `awake_since`: unix ts this awake period began. When the previous poll is from
    an earlier awake period, a session that started while asleep is dated from
    the wake-up rather than from that stale poll.
    """
    prev = state.get("prev", {})
    journey = state.setdefault("journey", {"active": False})
    charge = state.setdefault("charge", {"active": False})

    if not prev:
        state["prev"] = current
        return

    op_delta = current["op_time_s"] - prev.get("op_time_s", current["op_time_s"])
    charge_delta = current["charge_kwh"] - prev.get("charge_kwh", current["charge_kwh"])
    edge_ts = max(prev.get("ts", current["ts"]), awake_since)

    # ---- Journey state machine (operating_time = READY-mode seconds) ----
    driving_now = op_delta > 0
    if driving_now and not journey["active"]:
        # Started a new journey since last poll.
        journey.update({
            "active": True,
            "start_ts": edge_ts,
            "start_op_time": prev.get("op_time_s", current["op_time_s"]),
            "start_discharge": prev.get("discharge_kwh", current["discharge_kwh"]),
            "start_charge": prev.get("charge_kwh", current["charge_kwh"]),
            "start_soc": prev.get("soc", current["soc"]),
            "start_odo_mi": prev.get("odometer_mi", current.get("odometer_mi")),
        })
        mq.publish(f"{TOPIC_PREFIX}/journey/in_progress", "1", retain=True)
        log.info("journey START at SoC=%.1f%%", journey["start_soc"])
    elif journey["active"] and not driving_now:
        # Journey ended (no operating_time increase this cycle).
        end_journey(state, current, mq)

    # ---- Charge session state machine ----
    # Prefer the BMS "charging" flag; fall back to the cumulative-kWh delta when
    # that byte was lost (WiCAN-OBD-C3 drops leading ISO-TP frames).
    if current.get("charging_flag") is not None:
        charging_now = current["charging_flag"] == 1
    else:
        charging_now = charge_delta > CHARGE_DELTA_THRESHOLD_KWH
    if charging_now and not charge["active"]:
        charge.update({
            "active": True,
            "start_ts": edge_ts,
            "start_charge": prev.get("charge_kwh", current["charge_kwh"]),
            "start_soc": prev.get("soc", current["soc"]),
        })
        mq.publish(f"{TOPIC_PREFIX}/charge/in_progress", "1", retain=True)
        log.info("charge START at SoC=%.1f%%", charge["start_soc"])
    elif charge["active"] and not charging_now:
        end_charge(state, current, mq)

    state["prev"] = current


def get_12v_voltage():
    """Read 12V battery voltage from WiCAN /check_status."""
    try:
        with urllib.request.urlopen(f"http://{WICAN_HOST}/check_status", timeout=3) as r:
            data = json.loads(r.read())
        s = data.get("batt_voltage", "")
        # Format is "14.6V"
        return float(s.rstrip("Vv"))
    except Exception as e:
        log.debug("12V read failed: %s", e)
        return None


def decode_22_01_01_partial(resp_bytes: bytes, valid_offsets: set):
    """Extract fields from BMS 22 01 01 response where their byte offsets are
    all in `valid_offsets`. Fields located in lost CFs are omitted.

    Byte offsets are 0-indexed from start of response. Response begins with
    62 01 01 (SID+PID echo at offsets 0,1,2), then payload from offset 3.
    Cumulative fields live in the trailing payload — they survive the
    buffer-drop on WiCAN-OBD-C3.
    """
    out = {}

    def have(start, length):
        return all((start + i) in valid_offsets for i in range(length))

    if not have(0, 3):
        return out
    if resp_bytes[0] != 0x62 or resp_bytes[1] != 0x01 or resp_bytes[2] != 0x01:
        return out

    # Offsets calibrated empirically 2026-05-16 against this user's Ioniq 5
    # (76.5% SoC, raw response captured and bytes matched to plausible
    # lifetime totals). All u32 big-endian.
    #
    # NOTE: byte 7 (low nibble of FF + first CF byte) is the SoC display
    # byte (value/2 = %). It matches Mode 01 PID 5B exactly but is in a
    # buffer-drop-prone position, so we rely on PID 5B for live SoC instead.
    fields = [
        ("cumulative_charge_a_h",    33, 4, lambda d, o: int.from_bytes(d[o:o+4], "big") / 10.0),
        ("cumulative_discharge_a_h", 37, 4, lambda d, o: int.from_bytes(d[o:o+4], "big") / 10.0),
        ("cumulative_charge_kwh",    41, 4, lambda d, o: int.from_bytes(d[o:o+4], "big") / 10.0),
        ("cumulative_discharge_kwh", 45, 4, lambda d, o: int.from_bytes(d[o:o+4], "big") / 10.0),
        ("operating_time_s",         49, 4, lambda d, o: int.from_bytes(d[o:o+4], "big")),
    ]
    for name, ofs, ln, fn in fields:
        if have(ofs, ln):
            try:
                out[name] = fn(resp_bytes, ofs)
            except Exception as e:
                log.warning("decode %s: %s", name, e)
    return out


def do_poll(wican, state, mq, full, awake_since):
    """Run one set of OBD queries and publish. Only called while the 12 V says the
    car is awake. `full` adds the slow-changing PIDs (TPMS, odometer/VIN, DTE)."""
    published = 0
    soc_data = None
    soc = None
    resp = None
    valid = set()
    metrics = {}
    live = {}

    # 1) Single-frame SoC via Mode 01 PID 5B (BMS at 7EC).
    soc_data = wican.query_single(0x7DF, 0x7EC, bytes.fromhex("015B"))
    if soc_data:
        soc = decode_pid5b_soc(soc_data)
        if soc is not None:
            mq.publish(f"{TOPIC_PREFIX}/soc_pct", f"{soc:.1f}", retain=True)
            log.info("SoC = %.1f%%", soc)
            published += 1
    else:
        # BMS did not respond — ECUs are asleep. Stop here to avoid sending
        # further CAN frames that would wake additional ECUs and drain the 12V.
        log.info("PID 5B no response (ECU asleep) — aborting poll to protect 12V battery")
        return published

    # 1b) Standard OBD-II Mode 01 PIDs: ambient air temp (PID 46), vehicle speed (PID 0D).
    amb_data = wican.query_single(0x7DF, 0, bytes.fromhex("0146"))
    if amb_data:
        amb = decode_pid46_ambient(amb_data)
        if amb is not None:
            mq.publish(f"{TOPIC_PREFIX}/ambient_air_c", str(amb), retain=True)
            log.info("ambient = %d°C", amb)
            published += 1

    speed_data = wican.query_single(0x7DF, 0, bytes.fromhex("010D"))
    if speed_data:
        speed = decode_pid0d_speed(speed_data)
        if speed is not None:
            mq.publish(f"{TOPIC_PREFIX}/speed_kph", str(speed), retain=True)
            if speed > 0:
                log.info("speed = %d km/h", speed)
            published += 1

    # 2) Cumulative metrics from BMS via Mode 22 PID 0101.
    resp, total, valid = wican.query_iso_tp_late_cfs(
        0x7E4, 0x7EC, bytes.fromhex("220101"))
    if resp and len(valid) > 6:
        metrics = decode_22_01_01_partial(resp, valid)
        for k, v in metrics.items():
            mq.publish(f"{TOPIC_PREFIX}/{k}", f"{v:.1f}" if isinstance(v, float) else str(v), retain=True)
            published += 1
        log.info("cumulative metrics (%d/%d bytes): %s", len(valid), total, metrics)

    # 2b) Live + flag fields from BMS PID 0101.
    if resp and len(valid) > 6:
        live = decode_22_01_01_live(resp)
        for k, v in live.items():
            mq.publish(f"{TOPIC_PREFIX}/{k}", f"{v:.3f}" if isinstance(v, float) else str(v), retain=True)
            published += 1

    # 3) Battery health (SOH) + HV_KWH_R from BMS Mode 22 PID 0105.
    resp5, total5, valid5 = wican.query_iso_tp_late_cfs(
        0x7E4, 0x7EC, bytes.fromhex("220105"))
    if resp5 and all(o in valid5 for o in range(28, 30)):
        soh = decode_22_01_05_soh(resp5)
        if soh is not None:
            mq.publish(f"{TOPIC_PREFIX}/soh_pct", f"{soh:.1f}", retain=True)
            log.info("SOH = %.1f%%", soh)
            published += 1
    if resp5 and all(o in valid5 for o in range(31, 33)):
        extra5 = decode_22_01_05_extra(resp5)
        for k, v in extra5.items():
            mq.publish(f"{TOPIC_PREFIX}/{k}", f"{v:.2f}" if isinstance(v, float) else str(v), retain=True)
            published += 1
        if extra5:
            log.info("BMS extended: %s", extra5)

    # 3b-3d) Slow-changing values: once per SLOW_POLL_SECONDS while awake.
    if full:
        published += poll_slow(wican, state, mq)

    # 4) Mark the poll and drive the journey/charge state machines.
    if published:
        now_ts = int(time.time())
        mq.publish(f"{TOPIC_PREFIX}/last_seen", str(now_ts), retain=True)
        if (soc is not None and resp and len(valid) > 6
                and "operating_time_s" in metrics
                and "cumulative_discharge_kwh" in metrics
                and "cumulative_charge_kwh" in metrics):
            current = {
                "ts": now_ts,
                "soc": soc,
                "op_time_s": metrics["operating_time_s"],
                "discharge_kwh": metrics["cumulative_discharge_kwh"],
                "charge_kwh": metrics["cumulative_charge_kwh"],
                "odometer_mi": state.get("last_odometer_mi"),
                "charging_flag": live.get("charging"),
            }
            update_session_state(state, current, mq, awake_since)
            save_state(state)

    return published


def poll_slow(wican, state, mq):
    """TPMS, odometer/VIN and VMCU range. Only called once the BMS has already
    answered this poll, so the bus is known to be awake."""
    published = 0

    # 3b) Tyre pressures/temps from TPMS (7A0) PID C00B.
    resp_t, total_t, valid_t = wican.query_iso_tp_late_cfs(
        0x7A0, 0x7A8, bytes.fromhex("22C00B"))
    if resp_t and len(valid_t) >= 30:
        tyres = decode_22_c0_0b_tpms(resp_t)
        for k, v in tyres.items():
            mq.publish(f"{TOPIC_PREFIX}/{k}", str(v), retain=True)
            published += 1
        if tyres:
            log.info("tyres: %s", tyres)

    # 3c) Opportunistic odometer from cluster ECU (7C6, PID 22B002).
    if wican.open_extended_session(0x7C6, 0x7CE):
        if not state.get("vin"):
            vin_resp, vin_total, vin_valid = wican.query_iso_tp_late_cfs(
                0x7C6, 0x7CE, bytes.fromhex("22F190"), timeout=2.0)
            if vin_resp:
                vin = decode_vin(vin_resp)
                if vin:
                    state["vin"] = vin
                    mq.publish(f"{TOPIC_PREFIX}/vin", vin, retain=True)
                    log.info("VIN = %s", vin)
                    published += 1
        odo_resp, odo_total, odo_valid = wican.query_iso_tp_late_cfs(
            0x7C6, 0x7CE, bytes.fromhex("22B002"), timeout=2.0)
        if odo_resp and odo_resp[0] == 0x62:
            odo = decode_odometer(odo_resp)
            if odo:
                for k, v in odo.items():
                    mq.publish(f"{TOPIC_PREFIX}/{k}", str(v), retain=True)
                    published += 1
                if "odometer_mi" in odo:
                    state["last_odometer_mi"] = odo["odometer_mi"]
                log.info("odometer: %s", odo)
        elif odo_resp and odo_resp[0] == 0x7F and len(odo_resp) >= 3:
            log.debug("odometer NRC=0x%02X", odo_resp[2])

    # 3d) Predicted range (DTE) from VMCU (7E2 -> 7EA) PID 22 0101.
    resp_v, total_v, valid_v = wican.query_iso_tp_late_cfs(
        0x7E2, 0x7EA, bytes.fromhex("220101"))
    if resp_v and len(valid_v) > 6:
        log.info("VMCU 0101 raw (%d/%d): %s", len(valid_v), total_v, resp_v.hex())
        dte_km = decode_vmcu_dte(resp_v, valid_v)
        if dte_km is not None:
            mq.publish(f"{TOPIC_PREFIX}/dte_km", str(dte_km), retain=True)
            mq.publish(f"{TOPIC_PREFIX}/dte_mi",
                       f"{dte_km * 0.621371:.0f}", retain=True)
            log.info("DTE = %d km (%.0f mi) [uncalibrated]", dte_km, dte_km * 0.621371)
            published += 1

    return published


MAX_FAILED_POLLS = 3  # per awake period, then wait for the car to sleep and wake again


def main():
    log.info("wican-bridge starting (voltage-gated: poll every %ds while 12V >= %.1fV, "
             "slow PIDs every %ds, check every %ds)",
             POLL_SECONDS, AWAKE_VOLTAGE, SLOW_POLL_SECONDS, PRESENCE_CHECK_SECONDS)
    mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="wican-bridge",
                     protocol=mqtt.MQTTv311)
    mq.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mq.will_set(f"{TOPIC_PREFIX}/online", "0", retain=True)
    mq.connect(MQTT_HOST, MQTT_PORT, keepalive=120)
    mq.loop_start()
    mq.publish(f"{TOPIC_PREFIX}/online", "1", retain=True)

    state = load_state()
    awake_since = None  # unix ts the current awake period began; None while asleep
    last_poll = last_slow = 0.0  # monotonic
    failed_polls = 0
    stand_down = False  # BMS silent / repeated failures: no CAN until next awake period

    while True:
        # HTTP only — never touches the CAN bus. None = WiCAN asleep or out of range.
        v12 = get_12v_voltage()
        awake = v12 is not None and v12 >= AWAKE_VOLTAGE
        if v12 is not None:
            mq.publish(f"{TOPIC_PREFIX}/12v_battery_v", f"{v12:.2f}", retain=True)
        mq.publish(f"{TOPIC_PREFIX}/car_awake", "1" if awake else "0", retain=True)

        if not awake:
            if awake_since is not None:
                log.info("car asleep (12V %s) — polling stopped",
                         f"{v12:.2f}V" if v12 is not None else "WiCAN unreachable")
                end_sessions_on_sleep(state, mq)
                awake_since = None
            time.sleep(PRESENCE_CHECK_SECONDS)
            continue

        now = time.monotonic()
        if awake_since is None:
            awake_since = int(time.time())
            last_poll = last_slow = now - max(POLL_SECONDS, SLOW_POLL_SECONDS)
            failed_polls = 0
            stand_down = False
            log.info("car awake (12V %.2fV) — polling every %ds", v12, POLL_SECONDS)

        if not stand_down and now - last_poll >= POLL_SECONDS:
            full = now - last_slow >= SLOW_POLL_SECONDS
            wican = WiCAN(WICAN_HOST, WICAN_PORT)
            try:
                wican.connect()
                n = do_poll(wican, state, mq, full, awake_since)
                if n == 0:
                    # 12 V is up but the BMS didn't answer (e.g. surface charge just
                    # after switch-off). Don't keep probing a sleeping bus.
                    stand_down = True
                    log.info("BMS silent despite 12V %.2fV — no more polls until the car "
                             "sleeps and wakes again", v12)
                else:
                    failed_polls = 0
                    if full:
                        last_slow = now
                    log.info("poll complete (%d values%s)", n, ", incl. slow PIDs" if full else "")
            except Exception as e:
                failed_polls += 1
                stand_down = failed_polls >= MAX_FAILED_POLLS
                log.warning("Poll failed (%s)%s", e,
                            " — giving up until next awake period" if stand_down
                            else f" — retrying in {POLL_SECONDS}s")
            finally:
                wican.close()
                last_poll = now  # rate-limits retries too: never more than one poll per POLL_SECONDS

        time.sleep(PRESENCE_CHECK_SECONDS)


if __name__ == "__main__":
    main()
