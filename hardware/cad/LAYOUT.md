# Rig layout and CAD notes (PLAN 2.2)

Parametric CAD in `pendulum_cad.py` (CadQuery). `uv run python pendulum_cad.py` writes STEP + STL to `out/`;
`uv run python preview.py` renders `out/assembly_preview.png`. Change a number in `P`, re-run, re-print.

## Rig layout (decided 25 Sep: desk-edge beam)

- The rail is bolted to the **front face of a stiff beam** (e.g. 1.2 m of 18 mm plywood on edge) that is
  G-clamped along the **front edge of the desk**. The rods hang over the edge: the single needs 30 cm below
  the pivot and the double (both hanging) 50 cm; a ~72 cm desk clears both.
- The pendulum shaft comes straight out of the cart toward the room; **the rod swings in the frontmost plane
  of the rig** (Y ≈ 47 mm). Nothing else may be in front of Y ≈ 40 mm anywhere the rods can reach: at the rail
  ends the double's rods sweep up to 50 cm around the pivot.
- **So the motor and idler sit behind that plane:** the motor body goes behind the beam (end blocks overhang
  the beam ends), with the pulley in the belt plane. The belt runs just below the rail, between the beam and
  the carriage (Y ≈ −9 … −3 mm).
- Safety: keep legs and hands out of the swing zone in front of the desk; the E-stop must be reachable
  without reaching past the rods.

## Coordinates

X along the rail, Z up, Y out of the beam toward the room. Y = −13 beam face (MGN12H H = 13 mm);
Y = 0 carriage top face; Z = 0 rail centreline; the shaft is at Z = +34 mm.

## Parts so far (v1, untested)

| Part | Size (mm) | Est. print | Notes |
|---|---|---|---|
| cart | 70 × 49 × 84 | ~28 g | Plate on 12 mm pads on the MGN12H's 20 × 20 M3 pattern (M3 × **20**: engages 3.0 of the 3.5 mm thread). Boss holds two 608s ~13 mm apart. Ridged ledge for the belt below the carriage |
| belt_clamp | 40 × 17 × 3 | ~2 g | Squeezes the upper belt strand against the ledge; 2 × M3 × 12 self-tapping |
| sensor_mount | 36 × 14 × 36 | ~3 g | On 4 posts behind the plate; AS5047P board behind it, chip through a 14 mm window; slots ±2 mm |
| magnet_cap | Ø11 × 12 | ~1 g | Push-fit on the shaft's back end; holds the magnet 3 mm off the steel shaft |
| rod_hub | 36 × 18 × 14 | ~3 g | Split clamp on the shaft + socket for the 8 mm tube, M3 pinch screws |

**Steel shaft: cut to 49 mm.** The script prints this and the sensor gap; both follow from the parameters.

**Estimated cart mass ≈ 145 g** (printed ~35 g + MGN12H 54 g + two 608s ~24 g + 49 mm of 8 mm steel ~19 g +
sensor board and screws ~13 g). The sim assumed 400 g, and the AI's domain randomisation (±15%) never reaches
145 g: update `sim/params.yaml` cart.mass before the final training (weigh the real parts in PLAN 2.2).

## Why some choices were made

- **Sensor behind the plate:** the front of the shaft carries the rod, which swings a full circle, so no
  bracket can reach the front end. The back end sits in free space above the carriage.
- **Magnet cap (precaution):** the shaft is silver steel, which is magnetic and may distort the magnet's
  field; the AS5047P datasheet doesn't cover shaft material. The cap holds the magnet 3 mm off the steel.
  Check in PLAN 2.4 (smooth 0–360° reading, noise); set `cap_spacer` to 0 if it proves unnecessary.
- **Taller pads (M3 × 20):** gives 10.7 mm behind the sensor bracket for the board and its pins.

## Check when the parts arrive (before printing the final version)

- [ ] AS5047P adapter board: outline, hole positions, thickness, chip height, and header direction
      (update `sensor_mount` if the ±2 mm slots don't cover it).
- [ ] Test-print a 608 seat and the shaft bores; tune `fit` (default 0.15 mm radial).
- [ ] Measure the GT2 belt thickness; tune `belt_t`.
- [ ] PLAN 2.2 "done when": the cart slides the full rail by hand without binding, the shaft spins freely in
      both bearings, the magnet sits centred on the sensor.

## Still to design (PLAN 2.2)

Motor end block (motor behind the beam, 30T pulley in the belt plane), idler end block with belt tensioning,
end-stop mounts at both ends, E-stop enclosure, beam clamps.

Sources: HIWIN MG-series catalogue (MGN12H: W 27, B 20, C 20, L 45.4, H 13, M3 × 3.5, 0.054 kg; MGN12 rail
WR 12, HR 8, P 25): https://hiwin.co.uk/wp-content/uploads/2017/05/CRD-Hiwin-Linear-guides-MG-series.pdf ·
Pololu #4757 dimensions: https://www.pololu.com/product/4757
