# Electronics (PLAN 2.1) — power path, pin map, decisions

Written Fri 25 Sep 2026, before any wiring. Every figure below is quoted from a datasheet (sources at the end)
or measured in the sim (`sim/`). Decisions were made by Ollie from the options and evidence listed.

## 1. Power path

```
                        ┌──────────────── relay coil loop (signal current only, ~0.15 A) ────────────────┐
                        │                                                                                   │
 Mean Well   ┌───────┐  │  ┌─────────┐      P12      ┌──────────┐  N1  ┌──────────┐  N2   ┌──────────────┐  │
 GST60A12 +──┤ 7.5 A ├──┴──┤ E-stop  ├───────┬───────┤ end-stop ├──────┤ end-stop ├───────┤ relay coil   ├──┴── GND
 12 V 5 A    │ fuse  │     │ latching│       │       │ LEFT (NC)│      │RIGHT (NC)│       │ 12 V, 1N4007 │
             └───────┘     │ NC      │       │       └──────────┘      └──────────┘       │ across coil  │
                           └─────────┘       │                                            └──────────────┘
                                             │   relay contacts (NO, 30–40 A)
                                             └──────────────/ ──────────┐
                                                                        │
                                              Schottky ≥ 8 A, ≥ 40 V    ▼  (anode → cathode: energy can't flow back)
                                                                        │
                                   BUS ──────────────┬──────────────────┼──────────────┐
                                                     │                  │              │
                                              2200 µF 35 V        TVS 1.5KE22A     Cytron MD13S POWER+
                                              low-ESR (+ to BUS)  (cathode to BUS)  MD13S POWER− → GND
                                                     │                  │              │  MA/MB → motor red/black
 GND ────────────────────────────────────────────────┴──────────────────┴──────────────┴── common with Pico GND
```

- **Fuse first**, then the **latching E-stop**, then the split: the relay coil loop and the motor feed.
- **End-stops only carry the relay coil's current.** Either end-stop, or the E-stop, drops the relay and cuts
  motor power independently of software (SPEC §8). The switches never break motor current (micro switches
  have no usable DC power rating: Omron V-15 is 0.6 A at 125 V DC).
- **Schottky diode + capacitor + TVS (braking energy), decided 25 Sep:** braking pushes energy back into the
  supply. The GST60A12's over-voltage protection is 12.6–16.2 V and **latches off until re-powered**. In the
  sim, the worst 50 ms of returned energy is 164 mJ (single swing-up) and 146 mJ (double tour); peak reverse
  current 0.31–0.51 A. Keeping the PSU under 12.6 V without a diode would need ~27,000 µF. With the diode, the
  PSU never sees it; 2200 µF absorbs a 0.2 J burst with the bus rising to √(11.6² + 2·0.2/0.0022) ≈ 17.8 V,
  under the MD13S's 30 V maximum. The TVS (≈ 21–23 V breakdown) is a backstop clamp. The diode costs ~0.4 V;
  the sim still swings up 10/10 (single) and completes the double tour 5/5 at 11.6 V.
- **Firmware current limiter** (SPEC §11 Q6): supply current ≤ 4.5 A until Phase 4 fits R and Ke, then 5 A.

## 2. Logic wiring and pin map (Raspberry Pi Pico 2)

All the RP2350's digital GPIOs are **fault tolerant**: they accept up to 5.5 V *while the Pico is powered*
(IOVDD = 3.3 V), and start up with pull-downs. The ADC pins GP26–29 are **not** 5 V tolerant; keep them for
3.3 V signals only.

| Pico pin | Signal | Connects to | Notes |
|---|---|---|---|
| GP2 | Motor encoder A | Pololu yellow, via **1 kΩ series** | PIO quadrature needs A and B on consecutive pins |
| GP3 | Motor encoder B | Pololu white, via **1 kΩ series** | |
| VBUS (5 V) | Encoder Vcc | Pololu blue | Encoder needs 3.5–20 V, so 3.3 V is too low. VBUS goes up and down with the Pico's USB power; the 1 kΩ resistors limit current in the instant before the Pico's 3.3 V rail is up |
| GND | Encoder GND | Pololu green | |
| GP14 | Motor PWM | MD13S PWM, **10 kΩ pull-down to GND** | 20 kHz, sign-magnitude. PWM LOW = both outputs LOW = motor braked (MD13S truth table). The pull-down keeps it braked while the Pico is booting or unplugged (the manual doesn't state an internal pull-down) |
| GP15 | Motor DIR | MD13S DIR, **10 kΩ pull-down to GND** | |
| GND | Logic ground | MD13S GND | Shared ground (SPEC §8) |
| GP16 | SPI0 RX (MISO) | AS5047P #1 MISO (and #2 for the double) | The chip runs at 3.3 V with VDD and VREG tied (datasheet). ⚠️ **Not yet checked: how the AS5047P-TS_EK_AB adapter board is set up for 3.3 V vs 5 V** (read its operation manual on arrival, before powering it) |
| GP17 | SPI0 CSn #1 | AS5047P #1 CSn (joint 1) | |
| GP18 | SPI0 SCK | AS5047P #1/#2 CLK | Up to 10 MHz (AS5047P datasheet); start at 1 MHz over the joint-2 cable |
| GP19 | SPI0 TX (MOSI) | AS5047P #1/#2 MOSI | |
| GP20 | SPI0 CSn #2 | AS5047P #2 CSn (joint 2, double only) | Runs through the flexible joint-1 cable loop (PLAN 2.5b) |
| 3V3(OUT) | Sensor power | AS5047P VDD+VREG | |
| GP10 | Sense P12 | divider 10 kΩ / 3.3 kΩ from node P12 | HIGH = E-stop released |
| GP11 | Sense N1 | divider from node N1 | HIGH = left end-stop closed |
| GP12 | Sense N2 | divider from node N2 | HIGH = right end-stop closed |
| GP13 | Tapper servo | servo signal (PLAN 7.1) | Servo powered separately at 5 V, shared ground |
| GP25 | On-board LED | status | |

**End-stop sensing (voltage dividers, decided 25 Sep):** each divider gives 12 V × 3.3/(10 + 3.3) ≈ 2.98 V
(Pico logic HIGH is ≥ 2.0 V; each divider draws ~0.9 mA). They're fed from the PSU side of the diode, which
the PSU regulates at 12 V, so they never see the braking spike.

| P12 | N1 | N2 | Meaning |
|---|---|---|---|
| H | H | H | All clear, relay can be on |
| H | L | L | **Left end-stop hit** |
| H | H | L | **Right end-stop hit** |
| L | L | L | **E-stop pressed** (or no 12 V) |

**Logic level (direct + measure, decided 25 Sep):** the MD13S needs ≥ 3.0 V for HIGH; the RP2350 guarantees
only 2.62 V at full rated drive current (2–12 mA), though at the ~0.3 mA a 10 kΩ pull-down draws it sits near
3.3 V, and Cytron markets the MD13S for 3.3 V logic. **Check before any motor power:** with the PWM pin set
HIGH, measure ≥ 3.0 V at the MD13S input (and ≤ 0.5 V for LOW). If it reads low, add a 74AHCT125 5 V buffer.

## 3. Safety behaviour this wiring gives (feeds the PLAN 3.4 checklist)

- **Motor off at power-on and during reset:** PWM is pulled LOW, so the MD13S brakes the motor; the RP2350's
  pins also start with pull-downs.
- **E-stop and end-stops cut motor power with no software involved**, and the Pico can report which one.
- **Braking energy can't trip the PSU** (diode), and the bus is limited by the capacitor and TVS.
- Still to be proven on the bench, not assumed: every item in PLAN 3.4.

## 4. Parts added by this step (⚠️ prices not yet checked)

| Part | Spec | Qty | Notes |
|---|---|---|---|
| Schottky diode | ≥ 8 A, ≥ 40 V, e.g. MBR1045 / SR1045 (TO-220) | 1 | Needs a small heatsink or a copper area: ~0.4 V × 5 A peak |
| Electrolytic capacitor | 2200 µF, 35 V, low-ESR, 105 °C | 1 | Mind the polarity |
| TVS diode | Unidirectional, ~21–23 V breakdown, e.g. 1.5KE22A (through-hole) | 1 | Cathode to BUS |
| Resistors | 10 kΩ ×5, 3.3 kΩ ×3, 1 kΩ ×2 | – | Skip if a resistor kit is owned |

The relay (item 15a) and 1N4007 (15b) are already on the SPEC §6 list.

## Sources

- Cytron MD13S user manual v1.1 (Jun 2018): logic HIGH 3–5.5 V, LOW 0–0.5 V; PWM ≤ 20 kHz; truth table (PWM
  LOW → outputs LOW); 30 A current limit. https://www.farnell.com/datasheets/3759023.pdf
- Mean Well GST60A spec: GST60A12 12 V 5 A; overload 105–150% hiccup; over-voltage 12.6–16.2 V, "shut down
  o/p voltage, re-power on to recover". https://www.meanwell.com/Upload/PDF/GST60A/GST60A-SPEC.PDF
- RP2350 datasheet: VOH ≥ 2.62 V and VOL ≤ 0.5 V at IOVDD 3.3 V; VIH 2.0–5.5 V on fault-tolerant pins while
  IOVDD is powered; digital GPIOs are Digital IO (FT) with reset pull-downs.
  https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
- Pololu #4757: encoder Vcc 3.5–20 V; A/B outputs 0 V to Vcc; wires red/black motor, green GND, blue Vcc,
  yellow A, white B. https://www.pololu.com/product/4757
- AS5047P datasheet: SPI up to 10 MHz; 3.3 V operation with VDD and VREG tied.
  https://www1.futureelectronics.com/doc/ams/AS5047P-ATSM.pdf
- Sim: `sim/` regen-energy and 11.6 V checks (commands in the PR for this step).
