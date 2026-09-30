# Safety checklist 🚦 (PLAN 3.4, SPEC §9)

Every line must be **tested and seen working**, with the date and what was seen. **No closed-loop control (LQR,
swing-up, AI) until every line is ticked.** Run after `hardware/BRINGUP.md` stage 6. Commands run from `sim/`.
Start each motion test at the lowest duty given; the rod is on, the belt tensioned, hands clear, E-stop in reach.

| # | Check | How to test | Pass when | Date | Result / evidence |
|---|---|---|---|---|---|
| 1 | E-stop cuts power mid-motion and **brakes** | `bringup.py PORT spin --duty 0.15 --ms 3000`; press the E-stop mid-move | Cart stops sharply (motor shorted), `watch` shows ✗ P12 and the E-stop fault | | |
| 2 | Each end-stop drops the relay and brakes; the Pico says which | Push the cart by hand onto each end-stop ramp; then from ~150 mm away `spin --duty 0.15 --ms 1500 --no-soft-limit` towards each end (the soft limit would otherwise stop it first) | Relay drops, cart stops within ~120 mm, `watch` shows the left/right pattern from electronics.md | | |
| 3 | E-stop mid-motion also drops the relay | Same as 1: listen for the relay click | Relay drops with the E-stop | | |
| 4 | Speed-aware soft limit brakes before the end-stops | From centre, `spin --duty 0.2 --ms 1500` towards one end, then 0.3 | `watch`/printout shows "soft limit" and duty 0 **before** the end-stop trips | | |
| 5 | Heartbeat watchdog stops the motor within 100 ms | From ~250 mm off one end, `spin --duty 0.1 --ms 2000` towards the other; mid-move, in a 2nd terminal `pkill -9 -f bringup.py` (and separately: pull the USB) | Motor stops at once; film at 240 fps to time it (≤ 100 ms) | | |
| 6 | Hardware watchdog: a hang resets the Pico, motor off | `bringup.py PORT hang` | Prints PASS: the USB drops, the Pico reboots in IDLE, duty 0 | | |
| 7 | **Motor off on boot and on reset (no twitch)** | 12 V on, motor connected: unplug/replug the Pico's USB 5×; short RUN to GND (reset) 5× | Cart never moves | | |
| 8 | Soft travel limits stop the cart before the end-stops | Covered by 4 at both ends | Both ends | | |
| 9 | Fall detection in balance mode | `bringup.py PORT calibrate` (once), then `bringup.py PORT fallcheck` with the rod hanging | Prints PASS: "fell" latched from the first frame, peak duty 0 | | |
| 10 | Output clamp and slew limit active | `spin --duty 0.3 --ms 300`: read the printed duty | Duty ramps up by ≤ 0.25 per 2 ms frame and never exceeds the command | | |
| 11 | (Double only) rod-1 turn limit | Not in the firmware yet: add with the double (PLAN 8); the fault bit (7) is reserved | Before any double swing-up on hardware | | |
| 12 | Hands clear, rig clamped | The beam can't move when the cart hits a bumper by hand; a run area marked on the desk | – | | |

Signed off (all ticked): ____________  date: ________
