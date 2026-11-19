# Barcode analysis: what the supplied samples establish

Owner: Buddy 3. Code: `barcode.c` (symbology and capture
state machine), `drv_ir.c` (edge capture), `nav_map.c` (meanings),
`task_barcode.c` (task). Tests: `s3_unit_tests.c` S3-U01..U17.

## 1. Sources examined

| Source | What it contains | How it was measured |
|---|---|---|
| `Barcode & Line Specification.pdf` | Photo of a printed barcode beside the track line. Annotations: approx. barcode length **141 mm**, gap barcode→line **20 mm**, line width **18 mm** | Page rendered at high resolution; element run lengths measured along one horizontal scan line through the bars (dark/light threshold), x = 340..1400 px |
| `Barcode Sample.pdf`, pages 1–4 | Vector barcodes labelled "A" (pages 1–2) and "Z" (pages 3–4), two print scales | Pages rendered; run lengths measured on a horizontal line |
| `BARCODE.pptx` | Slides with barcodes labelled A, F, Z (EMF vector shapes) and W, X, Y (PNG images) | Slide XML unpacked; EMF bar geometry read, PNG run lengths measured |
| Project Briefing | Three IR sensors; the right one reads barcodes; barcodes encode navigation commands | Read |

All eleven measured element-width vectors are kept verbatim in
`s3_unit_tests.c` (`bc_vectors[]`) as test inputs.

## 2. Findings

Every sample, from all three sources, has the same structure:

* **29 elements** (bar, space, bar, …, bar) = 3 groups of 9 elements, with
  one extra narrow space between groups.
* Each group has **5 bars and 4 spaces**, and **exactly 3 of its 9 elements
  are wide**.
* There are only **two widths**, with a measured **wide:narrow ratio of
  about 3:1**: PDF 118–119 vs 39–40 px; PPTX 351 vs 117 units, 78 vs 26 px,
  66 vs 22 px, 27 vs 9 px. The photo of a real print gives 58–64 vs
  18–23 px; ink spread and blur make bars look wider and spaces narrower.
* The first and last group are identical in every sample: pattern
  `n w n n w n w n n` (`0x094`).
* The middle group differs with the printed label. "A" =
  `w n n n n w n n w` (`0x109`); "Z" = `n w w n w n n n n` (`0x0D0`); and so
  on for F, W, X, Y.

That is exactly the **Code 39** structure: 9 elements per character, 3 wide,
2 widths, `*` as start/stop (`0x094`). Each group is 6 × 1 + 3 × 3 = 15
units, so `*X*` with two 1-unit gaps is **47 narrow units**. The
specification's 141 mm then gives a **3.0 mm narrow** and **9.0 mm wide**
element, which matches the 3:1 ratio. The middle groups decode to the
printed labels, and only with the standard Code 39 table.

The symbology is therefore **derived from the samples, not assumed**. Other
common symbologies do not fit:

* **Code 128:** 6 elements per character in 4 widths.
* **EAN/UPC:** 4 widths, fixed guard patterns, digits only.
* **Interleaved 2 of 5:** characters in pairs, no `*` frame.

`barcode.c` builds its table from the Code 39 rule (bar pattern × wide-space
group) rather than from a typed list. S3-U01 checks the table: 44 unique
patterns, 3 wide each, and `*`, `A` and `Z` equal to the measured patterns.

The decoder supports every uppercase letter `A` through `Z` in standard
Code 39. It generates the letter patterns in three groups (`A`–`J`,
`K`–`T`, and `U`–`Z`) and matches the measured bars and spaces to those
patterns. A single-letter label such as `*B*` has 29 measured elements,
including the two inter-character gaps. The `*` symbols delimit the scan.

References

- [Zebra SM72 Integration Guide — ASCII Character Sets](https://docs.zebra.com/us/en/scanners/general/sm72-ig/ascii-character-sets.html)
- [Zebra ZPL Programming Guide — `^B3` Code 39](https://cpws.zebra.com/cpws/docs/zpl/zpl-zbi2-pm-en.pdf)

## 3. Buddy 3 navigation mapping

The current firmware uses these Buddy 3 mappings:

| Code 39 character | Buddy 3 command |
|---|---|
| A | Turn Right |
| W | Turn Left |
| Z | Turn Left |
| X | Straight |
| Y | U-turn |

`nav_map.c` loads these defaults at barcode-task startup. Other valid Code 39
characters are decoded and reported (telemetry fields 17–23, `EVT_BARCODE`)
but produce no motion unless explicitly mapped. S3-U15 verifies these
assignments.

* Navigation codes use 2 bits plus a separate valid bit (Tribunal 1,
  Buddy 3: "2-bit codes for up to 4 navigation commands").

Also open:

* Whether a command is executed **at once** (Week 6 statechart order
  decode → execute) or **at the next junction**. This is the switch
  `S3_NAV_AT_JUNCTION` in `subsystem3.h`, default 0 = at once.
* How far `STRAIGHT` drives through a junction: `S3_STRAIGHT_MM` = 120 mm
  `[CAL]`.

## 4. Why edge capture is needed (Tribunal 1, Version B change)

A 3 mm narrow bar under the sensor lasts **10 ms at 300 mm/s** and 7.5 ms
at 400 mm/s. The Week 6 plan polled the IR sensors every 20–50 ms, so whole
bars would be missed. That was the reason for Tribunal 1's change:

* **Current default:** digital GP26 is sampled every 1 ms using RP2040
  TIMER alarm 3. A new level must remain stable for 3 ms; the first
  candidate timestamp is retained so filtering does not add to element widths.
* **Optional capture modes:** GPIO edge capture when
  `S3_CAPTURE_POLL_1MS=0`, or ADC capture when `S3_BARCODE_ANALOG=1`
  with the 1 ms timer. These alternatives are retained; the current robot
  uses digital capture.
* Each recorded edge has a 2 us-resolution timestamp and a raw level in a
  64-entry single-producer/single-consumer ring. Overflow is reported through
  `SYS_FLT_BARCODE_OVERFLOW`. The barcode task owns decoding; it never
  controls motor hardware directly.
* `gpio_irq_count()` and the barcode task's serial diagnostics provide
  timing/edge observations. Measure actual rates on the configured robot;
  older estimates based on a 20-slot encoder do not describe its current setup.

## 5. Decoding method (`barcode.c`)

1. **Start:** a white-to-black edge after at least 30 ms of observed white.
   An initial black edge with no known previous level can also begin capture.
2. **Element widths:** elapsed microseconds between sensor transitions.
   Wheel speed and odometry are not decoder inputs. Keep movement reasonably
   steady within each symbol; relative-width classification tolerates print
   scaling, but arbitrary hand-pull acceleration can distort a symbol.
3. **Glitch merge:** an interior element shorter than 1 ms is merged with
   its neighbours.
4. **Classification:** compare bars with bars and spaces with spaces within
   each nine-element symbol. The minimum wide/narrow ratio is 1.3 for bars
   and 1.5 for spaces; ambiguous widths are rejected.
5. **Validation:** ordinary runs have 10k - 1 elements, valid patterns,
   `*` start/stop delimiters and narrow inter-character gaps. Both forward
   and reversed reading are supported. A 27-element `*<letter>*` scan can
   recover one merged group only when the uppercase-letter template match
   is unambiguous; this is not general acceptance of missing elements.
6. **End:** after at least one complete symbol has been observed, trailing
   white must last at least 100 ms and 2.5 times the reference widest
   element, capped at 3 s. A 27-element run uses the second-longest element
   for this reference. A capture with no edge for more than 3 s times out.
7. **Duplicates:** the same decoded first symbol within 2 s of the previous
   successful read is reported as `DUPLICATE` and produces no new command.
   There is no wheel-distance lockout.
8. **Navigation:** a valid mapped letter queues its instruction until a
   confirmed GP6 + GP2 junction. Later sensor edges cannot replace the
   pending instruction. Buddy 2 executes the movement and reports completion.
   During capture, line tracking continues at its configured reduced speed.
9. **Invalid input never produces a command.** It increments the invalid
   count, raises the degraded `BARCODE_INVALID` fault, and leaves normal
   line following in control.

## 6. Evidence and remaining tests

| Evidence | Result | Kind |
|---|---|---|
| S3-U02: all 11 supplied samples (PDF A/A/Z/Z, PPTX A/F/Z/W/X/Y, photo A) | all decode to their printed label | software test, real sample geometry |
| S3-U03/U04: reversed sample; 3.5× print scale | decode | software test |
| S3-U05..U09: wrong element count, no contrast, corrupted start, wide gap, missing start/stop | rejected | software test |
| S3-U10: drive simulation, 15 trials × 100/200/300 mm/s, ±15 % speed ripple, encoder speed 5 % high, 0.6 mm blur | 45/45 correct | **simulation** |
| S3-U11..U14: duplicate, line under sensor, stall, trailing quiet | as specified | simulation |
| INT-11: barcode → navigation → Buddy 2 turn/move → NORMAL | pass | simulation |

The table records earlier software evidence. After cleanup, physical checks
should cover GP26 polarity and capture, all printed barcode sizes, forward
and reversed reads, junction handoff, and return to line tracking. Host
simulation does not replace track testing or sensor calibration.
