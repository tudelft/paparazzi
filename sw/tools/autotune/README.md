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

- Sphinx user guide: `doc/sphinx/source/user_guide/airframe_autotune.md`
  (User Guide -> Airframe Auto-Tuning from Flight Logs)
- Standalone HTML: `doc/tools/autotune/airframe_autotune.html`
- Diagram sources (Graphviz): `doc/images/tools/autotune/*.dot`
