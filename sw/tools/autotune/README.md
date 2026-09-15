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
