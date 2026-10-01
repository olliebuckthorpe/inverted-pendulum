# Orders (PLAN 0.1)

Placed Mon 28 Sep – Thu 1 Oct 2026 by Ollie. Prices include VAT unless marked "ex VAT". Add order numbers and
delivery dates from the confirmation emails as they arrive.

## 1. TME — motor (£81.64 incl. £9.90 + VAT shipping)

| Part | As ordered | Qty | Price |
|---|---|---|---|
| Motor (SPEC 19) | Pololu #4757, 6.3:1 37Dx65L 12 V, 64 CPR encoder, helical pinion (TME: POLOLU-4757) | 1 | £58.13 ex VAT |

No cheaper genuine source on 29 Sep (Pololu direct $60.95 + shipping + import VAT; Opencircuit €68 ex VAT +
shipping). A JGB37-520 clone was simulated and rejected: see `sim/RESULTS-sim-phase.md` §"Clone motor check".

## 2. DigiKey UK — power, sensors, small electronics (≈ £78.40 incl. VAT, free delivery over £65 ex VAT)

Placed Thu 1 Oct. **The RS order of 29 Sep never completed** (no payment, no order in the RS account). On
re-ordering, RS had the AS5047P kit "temporarily out of stock" until **1 Dec 2026** (after the deadline):
Farnell lists it "No Longer Manufactured", Mouser had none, DigiKey had 392 but marks it "Discontinued at
DigiKey" (selling remaining stock). DigiKey was also cheaper for the Mean Well (£14.28 vs RS £22.36 ex VAT).
A third sensor (spare: discontinued part) plus a spare capacitor took the basket over £65, which costs less
than the £22 + VAT delivery.

| Part | As ordered | Qty | Price ex VAT |
|---|---|---|---|
| PSU (SPEC 9) | Mean Well GST60A12-P1J, 12 V 5 A | 1 | £14.28 |
| Angle sensor (SPEC 7) | ams-OSRAM AS5047P-TS_EK_AB adapter kit (magnet included) | **3** (joint 1, joint 2 for the double, 1 spare) | £42.84 |
| Schottky (15c) | onsemi MBR1045G, 45 V 10 A, TO-220AC | 1 | £2.08 |
| Capacitor (15d) | Panasonic EEU-FR1V222L, 2200 µF 35 V, FR low-ESR, 105 °C | 2 | £5.34 |
| TVS (15e) | Littelfuse 1.5KE22A (the design's original part) | 1 | £0.78 |
| Flyback (15b) | Vishay 1N4001-E3/54 | 3 | £1.23 |
| Resistors (15f) | Stackpole CF14JT, 0.25 W 5 %: 10 kΩ ×5, 3.3 kΩ ×3, 1 kΩ ×2 | 10 | ~£0.70 |

Subtotal £65.33 ex VAT, delivery free (UPS). **No mains lead ordered**: the GST60A12 has an IEC C14 inlet; use
any kettle/PC lead (C13).

## 3. Gears & Sprockets — drive (≈ £16.67 incl. £5.95 + VAT shipping)

| Part | As ordered | Part no. | Qty | Price ex VAT |
|---|---|---|---|---|
| Pulley (SPEC 4) | GT2 30T, 6 mm bore, 6 mm belt | 6001-013 | 1 | £2.19 |
| Belt (SPEC 2) | GT2 6 mm, 2.5 m open (same company as Technobots) | 6001-595 | 1 | £3.86 |
| Bearings (SPEC 11) | 608ZZ 8×22×7 | 4255-080 | 3 | £0.63 each |

## 4. Amazon UK — everything else (£129.87, free delivery)

| Part | As ordered | ASIN | Price |
|---|---|---|---|
| Rail (SPEC 1) | CNCMANS MGN12 1000 mm + MGN12H, "Green MGN12H" (eBay CNC-Planet wanted £87.95) | B0C8MKJP59 | £18.33 |
| Shaft (SPEC 12) | 8 × 330 mm silver steel, precision ground, BS1407 (unhardened: hacksaw-cuttable) | B01N210CXJ | £7.99 |
| Brake relay (15a) | JZK 2× JD1914 12 V 40 A 5-pin SPDT (30/87/87a) with harness socket | B09SWRQQKM | £7.19 |
| E-stop (SPEC 14) | mxuteuk 22 mm latching mushroom in a station box, 1NC 1NO, 10 A (AC rating only listed) | B088ZJMXNF | £9.99 |
| End-stops (SPEC 15) | Hailege 10× KW12-3 roller-lever micro switches (the 20×10×6.4 mm size the CAD carrier expects) | B07XGJ661V | £4.99 |
| Fuses (SPEC 16) | 6× inline standard-blade holders, 12 AWG, + 66 fuses incl. 7.5 A | B0DKX9R1FK | £7.99 |
| Idler (SPEC 5) | 10× GT2 20T 5 mm bore toothed idlers with dual bearings | B07SHCFYX1 | £11.49 |
| DC socket (SPEC 10) | RUNCCI-YUN 5.5 × 2.1 mm 10 A panel socket with leads | B0836N82PR | £7.99 |
| Power wire (SPEC 17) | 18 AWG silicone, tinned copper, 2.5 m red + 2.5 m black | B0CZRGH4CT | £7.91 |
| Jumpers (SPEC 17) | ELEGOO 120 Dupont (M-M, M-F, F-F) | B01EV70C78 | £6.99 |
| Breadboard (SPEC 17) | ELEGOO 3× 830-point | B01M0QJTI5 | £9.99 |
| M3 screws (SPEC 18) | 1040-pc M3 socket cap kit: 8/10/12/16/20/25/30/40 mm + nuts, washers | B0DPFQ8PM8 | £6.99 |
| Idler axle (SPEC 18) | 15× M5 × 40 A2 button head + nyloc + washers | B0FNW8HSZT | £6.69 |
| Switch screws (SPEC 18) | 900-pc M2 / M2.5 / M3 DIN912 stainless kit (lengths not listed) | B0F38B1CBD | £7.29 |
| Controller (SPEC 6) | Raspberry Pi Pico 2 W with headers (run as a Pico 2, radio off) | B0FC2QLC17 | £8.05 |

## 5. The Pi Hut — motor driver (£24.90 + delivery)

| Part | As ordered | Qty | Price |
|---|---|---|---|
| Driver (SPEC 8) | Cytron MDD10A, 10 A 5–30 V 2-channel (MD13S unobtainable in time, see hardware/electronics.md) | 1 | £24.90 |

## 6. Not yet bought (in person, when needed)

| Part | Spec | Needed for |
|---|---|---|
| Rod (SPEC 13) | Aluminium round tube, 8 mm OD, 1 mm wall, 1 m | The pendulum joint |
| Beam | 18 mm plywood, ~1.2 m × 150 mm, flat | Mounting the rail: **first** |
| Clamps | 2–3 G-clamps, opening ≥ desk + 18 mm | Holding the beam on the desk |
| Wood screws | ~30 × No.4 16–20 mm, 8 × No.6 25 mm | Rail, ramps, end blocks |
| Bumpers | Rubber door stops or dense foam (TPU not needed) | Before the first powered run |
| Tapper servo (SPEC 20) | Small hobby servo | PLAN 7.1 |
| Top rod (SPEC 21) | Carbon fibre tube, ~8 mm, 20 cm | The double |

## Checks on arrival

- **Rail:** 1000 mm with an MGN12**H** (45 mm) carriage; slide it end to end by hand: smooth, no gritty spots.
- **MDD10A:** marked **REV2.0** (30 V; Rev 1.0 is 25 V).
- **AS5047P kit:** read its operation manual for the 3.3 V vs 5 V setup **before powering it**.
- **Micro switches:** measure the mounting-hole size (M2 or M2.5) and the roller height above the body (the
  CAD assumes 4 mm, `hardware/cad/pendulum_cad.py` `switch_lever`).
- **Screw kit:** confirm M2 / M2.5 lengths of ~10–12 mm are included.
- **Relay:** pins labelled 30, 85, 86, 87, **87a**.
