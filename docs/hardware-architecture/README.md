# Hardware Architecture docs — index

Read **`v4-home-wifi-current.md`** first — it's the current network
architecture and the only one that matches the wiring/firmware in this
repo today. **`v5-ros2-bridge.md`** is additive, not a replacement —
read it once you want ROS2 on top of the v4 architecture (new, not yet
verified on hardware).

| File | Status |
|---|---|
| `v5-ros2-bridge.md` | **New, additive.** Optional ROS2 bridge node on the Pi; layers on top of v4, doesn't change it. Not yet run on hardware. |
| `v4-home-wifi-current.md` | **Current.** All boards on home WiFi. |
| `v3-pi-centric-ap.md` | Superseded 2026-07-20. Pi hosted its own WiFi AP. |
| `v2-pi-integration-phase.md` | Superseded. Still has valid BNO055/gimbal/power wiring notes. |
| `motor-driver-tb6612fng.md` | Not part of the network-topology chain above — TB6612FNG motor driver wiring, still current. |
| `v1-original-as-built/` | Original pre-TB6612 (BTS7960-era) architecture PDFs. Pure history. |

Each versioned doc says at the top what it's superseded by, if anything.
