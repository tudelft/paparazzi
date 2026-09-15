# Paparazzi Airframe Auto-Tuning

`log_2_tuned_airframe.py` reads a Paparazzi flight log (`.log` + `.data`) and, from
**AUTO1 (stabilized) flight alone**, derives pitch/roll command trims, ruddervator
yaw mixing, and the ETECS energy-controller plant coefficients (cruise throttle and
speed, throttle/pitch per climb rate, max climb, glide ratio). The course loop is
model-checked from AUTO1 and tuned from data when >= 60 s of AUTO2 is present.
It writes a version-numbered tuned copy of the airframe XML (`<airframe>_optim_NNN.xml`),
compiles it, and prints an audit report with a before/after/why for every change plus
advice for the next flight.

```bash
./log_2_tuned_airframe.py --log-name 26_09_05__18_42_28 --ac-id 129 [--turn-radius 30] [--base prev_optim.xml] [--no-build]
```

Full documentation:

Use `--auto1-only` to exclude AUTO2 and a three-second margin on either side of
mode transitions. Course gains remain unchanged in this mode. The embedded
airframe in the log is the source for flown hardware settings and mixer checks;
`--base` is the current XML to preserve when preparing a new proposal, including
fixes made since the flight. Explicit `--set NAME=VALUE` choices take precedence.

Use `--logged-base` instead of `--base` when the candidate must start from the
exact embedded airframe. Attribute case/order and whitespace are normalized for
editing; logged settings, targets and mixers are retained. The current airframe
is not promoted or overwritten permanently. `--reviewed-only` disables heuristic
edits and applies only explicitly reviewed `--set` values, while still producing
flight diagnostics and a build check. For example:

```bash
python3 sw/tools/autotune/log_2_tuned_airframe.py \
  --log-name 26_09_15__13_56_59 --ac-id 129 --logged-base --reviewed-only \
  --turn-radius 32 --set ENERGY_BANK_WASHOUT_GAIN=0.65
```

State `AIRSPEED` is preferred over lower-rate `AIR_DATA`; course angles are
unwrapped before interpolation. Plant estimates use settled AUTO1 even in a
mixed-mode analysis. A full-throttle level run is not a maximum-climb test.
The single-circle RMS is withheld for changing centers such as an oval.
The bank-washout diagnostic replays the logged bank-only throttle transient on a
0.2 s grid, using AUTO2 as an energy-controller activity proxy. It reports strong
negative-term duration, underspeed fraction and median throttle contribution.
Internal-rate motion and initial filter state are unobserved; this is not a
closed-loop prediction. Reducing washout weakens both roll-in boost and roll-out
subtraction. The example above is a reviewed flight trial, not a general default.
Manual crow is not observable in `COMMANDS.BRAKE` with the direct-RC mixer;
check receiver or actuator telemetry before using AUTO1 samples for identification.

Flight-derived trims and plant estimates are not flight validation. Level pitch
does not identify IMU mounting error independently of angle of attack, and this
tool does not identify pitch P/D gains or certify stall speed. Reject estimates
from oscillatory or changing flight conditions; do not tune cruise performance
from brake-on AUTO2 data.

Run regression tests with:

```bash
python3 -m unittest discover -s sw/tools/autotune -p 'test_*.py' -v
```

- Sphinx user guide: `doc/sphinx/source/user_guide/airframe_autotune.md`
  (User Guide -> Airframe Auto-Tuning from Flight Logs)
- Standalone HTML: `doc/tools/autotune/airframe_autotune.html`
- Diagram sources (Graphviz): `doc/images/tools/autotune/*.dot`
