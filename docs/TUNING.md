# Tuning log

None of the constants in this project were guessed. Each one came out of a
measurement run on the physical robot. This file records what was measured, what
the iteration history actually looked like, and what went wrong on the way.

## The drive tuning session

Nine versions of the drive controller, all written in a single day. The sketches
are in `esp32/tuning_history/`.

| Time | Version | Lines | What changed |
|---|---|---|---|
| 11:39 | `v4` | 277 | Base controller |
| 14:24 | `v5_distance_fix` | 384 | Distance calculation corrected |
| 14:40 | `v6_distance_antistall` | 449 | Anti-stall handling added |
| 16:22 | `v7_encoder200_retune` | 452 | Forward drive retuned — end of the forward work |
| 16:41 | `v8_rawcount_retune` | 446 | Attempt to simplify to raw counts. Did not hold |
| 17:08 | `v10_v6logic_encoder200` | 467 | Reverted to v6's logic with the retuned encoder |
| 17:58 | `v11_backward` | 484 | Reverse drive begins |
| 18:07 | `v12_backpower` | 490 | Separate power curve for reverse |
| 18:41 | `v15_backtune` | 516 | Latest reverse tuning |

Two things in that table are worth pointing at.

**v8 shrank.** It was an attempt to simplify the distance logic down to raw encoder
counts, and the file getting smaller is the attempt itself. It did not work on the
robot, and v10 — named `v6logic` — is the revert to the approach that did.

**Everything from v11 onward is the reverse-drive problem**, which is still the
unfinished module. Forward drive was settled at v7 and has not needed changing
since.

## Drive and odometry

| Parameter | Value | Notes |
|---|---|---|
| `PULSES_PER_CM` | 7.52 | Measured over repeated fixed-distance runs |
| `GLITCH_NS` | 200 | Encoder debounce — below this, quadrature noise inflated the count |
| `stopEarly` | 1.00 cm | Overshoot after the stop command |
| `speedCoast` | 0.010 | Coast compensation |
| `turnLoss` | 0.25 | Distance lost to steering correction during a straight run |

The encoder count had to be trusted first, because every X position downstream
derives from it. Early versions commanded a distance and assumed it was travelled.
The final version reads `cm_done` back from the ESP32 after every move and uses the
measured value, because the two differ.

## Forward motion (settled at v7)

| Parameter | Value |
|---|---|
| `cruisePower` | 140 |
| `kickP` / `kickCm` | 175 / 4.0 |
| `rampDn` | 6 |
| `crawl` | 125 |
| `minMove` | 120 |
| `unstick` | 185 |
| `gain` | 12.0 |
| `smooth` | 0.30 |

`minMove` and `unstick` exist because the motors stall below a power threshold
under load and need a brief higher-power pulse to break static friction. A single
cruise value does not work.

## Reverse motion (in progress)

| Parameter | Value |
|---|---|
| `backScale` | 1.40 |
| `backCruise` | 110 |
| `backGain` | 16 |
| `backSteerSign` | +1 |

Reverse needed its own parameter set. The drivetrain is not symmetric — the same
power produces a different distance backwards than forwards, hence `backScale` —
and line correction needs a higher gain to stay stable. Four versions went into
this and it is the module still being worked on.

## Line following

| Parameter | Value |
|---|---|
| `polarity` | 0 |
| `center` | 0 |
| Servo neutral | 86 |
| Servo max left / right | 98 / 74 |

Steering is servo-driven, and the mechanical centre is not the numerical centre of
the servo range — 86, not 90. Line correction runs inside every drive command
rather than as a separate mode, so the robot tracks the tape continuously while
covering a commanded distance.

## Z mast

Stepper-to-distance measurements taken during calibration:

```
 391 steps  ->  19.0 cm
 806 steps  ->  39.5 cm
2430 steps  -> 124.9 cm   (mast visibly curved at this height)
```

The first two are linear and consistent. The third is not — at that extension the
mast flexes, so positions near the top of the range are less accurate than
positions near the bottom. A mechanical limit, not a software one, and it sets the
practical ceiling on shelf height.

## Scan geometry

| Parameter | Value | Reason |
|---|---|---|
| `SCAN_STEP_CM` | 12.0 | Smaller than the camera's view, so consecutive photos overlap |
| `OVERLAP_MERGE_FACTOR` | 1.25 | 15 cm merge window |

Overlap is deliberate. The robot moves less than one field of view between
captures so no book falls in a seam, which means the same book appears in two
photos. Detections on the same row within the merge window are treated as one
physical book and the higher-confidence result is kept.

## Failures worth recording

**Bluetooth debug output broke command handling.** The ESP32 prints human-readable
logs before its JSON reply. The Pi parsed the first line it received, saw malformed
JSON, and retried — issuing a second movement command. Fixed by skipping non-JSON
lines and waiting for the real response.

**A lost line looked like a successful drive.** If the sensors lost the tape
mid-move the ESP32 stopped but reported completion, and every X position after that
was wrong with no indication. It now returns an error and the scan halts.

**Z calibration ran longer than needed.** The original logic waited for the highest
tag to leave the frame. It now stops once every shelf-level tag has enough samples.

**One bad frame killed the whole scan.** A JPEG that failed to decode, or a detector
exception, aborted the run. Both are now caught and logged, and the scan continues.
