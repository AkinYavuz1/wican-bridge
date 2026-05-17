# wican-bridge

OBD telemetry bridge for Hyundai/Kia **E-GMP** electric vehicles (Ioniq 5/6, EV6, Niro EV, etc.). Pulls live state-of-charge, lifetime energy totals, battery health, cell-level voltage/temperature spread, live pack V/A/W, tyre pressures, and odometer directly from the car's CAN bus via a **WiCAN-OBD-C3** dongle, and publishes them as MQTT topics ready for Home Assistant, Grafana, or anything else.

No cloud. No BlueLink. No vendor app. Works on home WiFi.

## Why

The BlueLink app gives you ~5 metrics on a 30-minute lag. Direct OBD gives you 30+ metrics every 15 minutes, including the ones that actually matter for battery longevity (cell spread, pack temperature delta, SOH). This bridge was built and tuned against a real 2021 Ioniq 5 (72 kWh, 48k mi).

## What you need

- A Hyundai/Kia E-GMP vehicle (or anything with the same BMS PIDs — Ioniq 5/6, EV6, GV60, Niro EV)
- A **WiCAN-OBD-C3** dongle (https://www.crowdsupply.com/meatpi-electronics/wican-obd) — ~£40
- An always-on MQTT broker on your home network (Mosquitto)
- Docker

## Published MQTT topics

Under `TOPIC_PREFIX` (default `ioniq5`):

| Topic | Description | Typical value |
|---|---|---|
| `soc_pct` | State of charge | `52.9` |
| `soh_pct` | State of health (battery degradation) | `100.0` |
| `cumulative_charge_kwh` | Lifetime kWh into pack | `20486.9` |
| `cumulative_discharge_kwh` | Lifetime kWh out of pack | `19588.2` |
| `cumulative_charge_a_h` / `cumulative_discharge_a_h` | Lifetime Ah | |
| `operating_time_s` | Lifetime READY-mode seconds | `61334109` |
| `hv_kwh_remaining` | Pack energy remaining | `38.4` |
| `hv_pack_v` / `hv_pack_a` / `hv_pack_w` | Live pack voltage/current/power | |
| `hv_t_min` / `hv_t_max` | Pack temperature extremes (°C) | |
| `hv_cell_v_min` / `hv_cell_v_max` / `hv_cell_v_diff_mv` | Cell voltage spread | |
| `charging` / `charging_dc` / `ac_plug` | Charge state flags | |
| `tyre_p_fl_psi` ... `tyre_p_rr_psi` | TPMS pressures | `36.8` |
| `tyre_t_fl_c` ... `tyre_t_rr_c` | TPMS temperatures | `17` |
| `odometer_mi` | Odometer (miles) | `48227` |
| `vin` | Vehicle ID (one-shot) | |
| `12v_battery_v` | 12 V auxiliary battery | `12.30` |
| `journey/in_progress`, `journey/last_*` | Trip summaries (auto-detected) | |
| `charge/in_progress`, `charge/last_*` | Charge session summaries (auto-detected) | |
| `online` | Retained 1/0 LWT | |
| `last_seen` | Unix ts of last successful poll | |

## Quick start

```bash
git clone https://github.com/AkinYavuz1/wican-bridge.git
cd wican-bridge
cp .env.example .env
$EDITOR .env   # set MQTT_HOST, MQTT_USER, MQTT_PASSWORD, WICAN_HOST
docker compose -f docker-compose.example.yml up -d --build
docker logs -f wican-bridge
```

You should see:
```
INFO wican-bridge starting (poll every 900s)
INFO WiCAN slcan opened (192.168.0.12:3333)
INFO SoC = 52.9%
INFO cumulative metrics (34/62 bytes): {'cumulative_charge_kwh': 20486.9, ...}
INFO odometer: {'odometer_mi': 48227}
```

## WiCAN dongle setup

1. Plug WiCAN into the OBD port. The car must be at least in ACC for it to power up.
2. Connect your phone to the WiCAN's broadcast WiFi (`WiCAN_xxxx` SSID).
3. Open the WiCAN web UI (default `http://192.168.4.1`).
4. Configure it to join your home WiFi.
5. Set protocol to **slcan over TCP** on port `3333`.
6. Set CAN bitrate to `500 kbit/s`.
7. (Optional) Reserve a static DHCP lease for it on your router.

Note: WiCAN is home-WiFi only — it doesn't have cellular. You only get data when the car is in WiFi range. That's by design and fine for daily driving + home charging telemetry.

## Why the InfluxDB schema in `telegraf.example.conf` is weird

Default Telegraf MQTT consumer config writes every payload to a generic `value` field with the topic as a tag — making queries painful. The included starlark processor pivots topic-suffixes into proper field names so each metric has its own InfluxDB field (e.g. `soc_pct`, `odometer_mi`). The included Grafana dashboard expects this schema.

## Grafana dashboard

`grafana/ioniq5_car.json` — import directly. Expects an InfluxDB v2 data source UID of `influxdb-energy` pointing at a bucket named `energy`. Edit the JSON if yours differs.

Panels: SoC, remaining kWh, SOH, 12V battery, charge/plug/drive status, pack temperatures, cell V spread, tyre pressures, live pack V/A/W, cumulative kWh in/out, last journey summary, last charge summary, stale-data alert.

## Technical notes

The WiCAN-OBD-C3's slcan TX buffer is too small to deliver the leading consecutive frames of a multi-frame ISO-TP burst. We can't fix that in software — the ECU ignores ISO-TP flow control and bursts anyway. So:

1. **SoC** comes from standard OBD-II Mode 01 PID 5B, which fits in a single CAN frame. No ISO-TP, no buffer issue.
2. **Cumulative lifetime totals** come from Mode 22 PID 0101. We lose the leading CFs but the *trailing* ones reliably arrive, and the fields we care about (cumulative kWh, operating time) live in those trailing bytes. The bridge reassembles partial responses and only emits fields whose byte offsets all survived.
3. **Odometer** comes from cluster ECU (`0x7C6` → `0x7CE`) Mode 22 PID B002 after opening an extended diagnostic session. Calibrated against a real dashboard reading.

If you're porting this to a non-E-GMP car, expect to recalibrate byte offsets in `decode_22_01_01_partial` and `decode_odometer`.

## Caveats

- **Newer cars are locking this down.** 2024+ Hyundai/Kia models may use CAN-FD with gateway-restricted PIDs. Tested on 2021 Ioniq 5.
- **Don't expect 100% poll reliability.** ECUs sleep when the car is parked and unplugged. The bridge handles this gracefully (logs "no response" and retries).
- **No cloud, no app.** This is a self-hosting tool. If you want push notifications or remote access, pair it with Home Assistant + Tailscale.

## License

MIT.
