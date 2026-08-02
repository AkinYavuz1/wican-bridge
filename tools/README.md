# tools/

Standalone utilities that use the WiCAN's slcan/TCP interface but are
independent of `bridge.py`'s Hyundai/Kia E-GMP decoding. Not run by
`docker-compose.example.yml` — invoke directly with `python3`.

## obd_dtc_reader.py

Generic OBD-II diagnostic trouble code (DTC) reader. Works on any
OBD-II-compliant vehicle (petrol, diesel, or EV) since it only speaks
standard SAE J1979 modes — no vehicle-specific decoding.

```bash
pip install --no-deps -r <(echo "")   # no extra deps beyond stdlib
python3 tools/obd_dtc_reader.py --host 192.168.4.1
```

Reads, in order:
1. MIL (check engine light) status + confirmed DTC count (Mode 01 PID 01)
2. VIN (Mode 09 PID 02)
3. Stored DTCs (Mode 03) and pending DTCs (Mode 07) — from every ECU that
   answers the broadcast request, not just the engine
4. Freeze frame snapshot (Mode 02) if any DTC is stored — RPM, coolant
   temp, load, speed, fuel trims, etc. at the moment the first fault was set

Flags:
- `--json out.json` — also dump everything to a JSON file
- `--clear` — send Mode 04 (clear all DTCs) after an interactive
  confirmation prompt. **This also resets emissions readiness monitors** —
  note the codes down first, the car may need a drive cycle before it
  reports "ready" again.
- `--no-vin` / `--no-freeze-frame` — skip those steps
- `--custom-req-id 0xNNN --custom-resp-id 0xNNN` — query a non-engine
  module (BSI, ABS, airbag, etc.) via UDS `ReadDTCInformation` (service
  0x19). See caveat below.

### Generic vs manufacturer-specific codes

Only the engine/powertrain ECU is guaranteed reachable via the standard
broadcast ID (`0x7DF` → `0x7E8`-`0x7EF`). The script includes a lookup
table for common **generic** `P0xxx`/`U0xxx` codes (SAE-standardized,
identical meaning on every make). Manufacturer-specific codes
(`P1xxx`/`C1xxx`/`B1xxx`/`U1xxx`, or anything from a non-engine module)
print with no description — those need to be looked up against that
manufacturer's fault code list.

### Reading a non-engine module (e.g. Peugeot BSI)

Modules like the BSI (body control computer) on PSA/Peugeot-Citroën
vehicles aren't on the standard broadcast OBD-II address — they sit on a
manufacturer-specific diagnostic CAN ID that varies by model and
generation. This script does **not** hardcode a guess at that ID (getting
it wrong risks talking to the wrong module or getting silence). To read
BSI faults you need to find the correct request/response arbitration IDs
first — via PSA-specific documentation/forums, or by sniffing the CAN bus
while triggering a BSI-controlled function (indicators, central locking,
etc.) to see which ID reacts — then pass them with `--custom-req-id`/
`--custom-resp-id`. The script will decode the raw DTC numbers via the
standard UDS format, but their *meaning* is still PSA-proprietary — you'll
likely need to cross-reference them against a PSA fault code list (e.g.
Lexia/Diagbox documentation or community-maintained lists) to interpret
what's actually wrong.

### Workflow

This runs on your machine on the WiCAN's WiFi network — it doesn't run in
a cloud session. Typical loop: run the script, paste its output (or the
JSON file) back into your conversation for interpretation/diagnosis.
