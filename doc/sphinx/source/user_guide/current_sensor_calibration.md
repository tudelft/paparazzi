# Better Current Readings with Simple Bench Calibration

Current sensors built into flight controllers are convenient, but they are not always especially accurate at the currents used by small aircraft. A sensor may look perfect near idle and then report much too little current as the motor spins faster. That affects the consumed-capacity counter, remaining-flight-time estimates, battery warnings, logs, and any feature that uses electrical power data.

Paparazzi can correct this with one practical bench measurement. You do not need to calculate a polynomial, identify a shunt resistor, or become close friends with a spreadsheet. Record the current in amperes shown at **Messages → ENERGY → current (A)**, record the current in amperes shown on an RC watt meter at the same moment, and put those two values in the airframe file.

The result is a current reading that stays much closer to reality across the useful range. Better data means more trustworthy battery decisions, and batteries are famously poor at accepting apologies after landing.

## What You Need

- The aircraft with its normal flight controller and power wiring.
- The battery type normally used by the aircraft.
- The aircraft's normal motor and propeller.
- An RC watt meter with suitable connectors. This is a small display box placed between the battery and aircraft. Despite its name, it measures more than watts: it also shows battery voltage, current in amperes, power in watts, and often consumed capacity in amp-hours. Its current rating must be higher than the maximum current expected from the motor.
- A working telemetry connection to Paparazzi Center.
- A safe way to fasten the aircraft to a solid bench so it cannot move when the motor runs.

The RC watt meter is the reference for this test. One of the most commonly sold examples is the **G.T.Power 130A Watt Meter and Power Analyzer**, also sold in many very similar 130 A versions under other brand names. You do not need that exact model; any suitable RC watt meter that displays current in amperes will work. Check its manual if its battery and aircraft sides are not clearly marked.

:::{important}
Disconnect the battery before changing any wiring. Insert the watt meter **between the battery and the aircraft**: battery connector to the meter's input, then the meter's output to the aircraft. Never connect the watt meter directly between the positive and negative battery terminals. Confirm that the meter, wires, and connectors are rated above the expected motor current before applying power.

For a motor test with a propeller fitted, work in a clear test area, restrain the aircraft mechanically, stay outside the propeller arc, keep other people away, and have a second person ready to disconnect power. Do not rely on holding the aircraft by hand.
:::

## The Three Values

The calibration uses two required values and one optional value:

| Airframe define | What it means |
| --- | --- |
| `CURRENT_ADC_CALIBRATION_REPORTED` | The current in amperes shown by Paparazzi at the chosen steady throttle setting. |
| `CURRENT_ADC_CALIBRATION_ACTUAL` | The current in amperes shown by the RC watt meter at exactly the same throttle setting. |
| `CURRENT_ADC_CALIBRATION_ACCURATE_TO` | Optional: the highest current in amperes below which Paparazzi is already accurate enough and should remain unchanged. |

All three values are in amperes. There are no hidden coefficients: the numbers in the airframe file are the numbers you observed on the bench.

## Step-by-Step Calibration

### 1. Prepare the aircraft

Use the normal battery, flight controller, wiring, ESC, motor, propeller, and onboard equipment. Fasten the aircraft securely to a solid bench with the propeller in clear air. Keep all straps, cables, tools, and body parts away from the propeller.

With the battery disconnected, insert the watt meter between the battery and aircraft. Connect the battery to the meter's input and the aircraft to its output. Check connector polarity before applying power.

Start Paparazzi Center and connect telemetry. Open the **Messages** tool, find the **ENERGY** message, and watch **current (A)**. This is the Paparazzi current reading used throughout this guide.

### 2. Check zero and the low-current region

Apply power with the motor stopped and allow both readings to settle. Compare **ENERGY → current (A)** with the current in amperes shown on the watt meter. Both should show a plausible value for the powered flight controller, receiver, telemetry radio, and other onboard electronics.

Increase the throttle slowly and compare the two current readings. If they remain close up to a useful current, write down that current in amperes as `CURRENT_ADC_CALIBRATION_ACCURATE_TO`.

For example, if Paparazzi and the meter agree well through approximately 0.50 A, use:

```xml
<define name="CURRENT_ADC_CALIBRATION_ACCURATE_TO" value="0.50" unit="A"/>
```

This value is optional. Leave it out when the sensor needs correction from zero, or when you have not verified an accurate low-current region.

### 3. Choose one strong calibration point

Slowly raise the throttle until the motor is drawing a current near the upper end of what it normally uses in flight. A current around 70-100% of the expected maximum is ideal, provided the motor, ESC, battery, wiring, and meter remain within their continuous ratings.

Do not record the brief current jump when the motor starts. Keep the throttle at one unchanged position for several seconds. Wait until the current on the watt meter and **ENERGY → current (A)** are reasonably steady. If the last digit moves slightly, use a sensible middle value.

At the same moment, write down:

1. **Paparazzi current in amperes:** the value shown at **Messages → ENERGY → current (A)**.
2. **Actual current in amperes:** the value shown on the RC watt meter.

Example bench notes:

```text
Messages -> ENERGY -> current (A): 3.30 A
RC watt meter current:             4.60 A
```

These become:

```xml
<define name="CURRENT_ADC_CALIBRATION_REPORTED" value="3.30" unit="A"/>
<define name="CURRENT_ADC_CALIBRATION_ACTUAL" value="4.60" unit="A"/>
```

### 4. Add the values to the airframe

Place the defines in the battery or electrical section of the airframe file. A complete example is:

```xml
<section name="BAT">
  <!-- Accurate through 0.50 A. At the high bench point Paparazzi reported
  3.30 A while the RC watt meter showed 4.60 A. -->
  <define name="CURRENT_ADC_CALIBRATION_ACCURATE_TO" value="0.50" unit="A"/>
  <define name="CURRENT_ADC_CALIBRATION_REPORTED" value="3.30" unit="A"/>
  <define name="CURRENT_ADC_CALIBRATION_ACTUAL" value="4.60" unit="A"/>
</section>
```

Rebuild and flash the aircraft. Paparazzi prints the active calibration values during the build, making it easy to confirm that the intended airframe settings were used.

## What Paparazzi Does with the Values

Below `CURRENT_ADC_CALIBRATION_ACCURATE_TO`, the original reading is kept. Above it, Paparazzi applies a straight-line correction through the reported and actual bench values. Readings above the calibration point continue along the same correction line.

This approach deliberately favors clarity and robustness over a complicated curve fit. One good simultaneous measurement usually removes most of the useful-range error while keeping future adjustments understandable.

## Verify the Result

Repeat the bench test after flashing the calibrated firmware:

1. Check a low-current point.
2. Check a middle-current point.
3. Check the high calibration point.

At the high point, **ENERGY → current (A)** should now be close to the current in amperes shown by the watt meter. The middle point should also improve substantially. Small differences are normal because battery voltage, temperature, ADC noise, ESC behavior, and meter update rates all move slightly during a test.

For small aircraft, an error within roughly 0.1-0.2 A over a 0-5 A range is often already a useful result. Choose a tighter acceptance limit when capacity accounting or power monitoring requires it.

If the corrected reading becomes less accurate in the middle of the range, repeat the test. Keep the throttle unchanged for several seconds and confirm that both current readings were written down at the same moment. Sensors with severe or irregular nonlinearity may need calibration closer to the throttle setting most often used in flight.

## Updating the Calibration Later

Recheck the calibration after changing the flight controller, current-sensor hardware, power wiring, or ADC scaling. A different motor or propeller does not normally require sensor recalibration, but it may change the current range, so confirm that the original calibration point still represents normal operation.

Keep the bench values in a short XML comment. Six months later, `reported 3.30 A, actual 4.60 A` is much more helpful than an unexplained gain copied from an old notebook.

## No Current ADC?

The reported/actual calibration applies only to a real ADC current measurement. If the aircraft has no current sensor, Paparazzi can estimate current from throttle using `MILLIAMP_AT_IDLE_THROTTLE` and `MILLIAMP_AT_FULL_THROTTLE`.

When the full-throttle value was measured at a known voltage, record that physical voltage too:

```xml
<define name="MILLIAMP_AT_IDLE_THROTTLE" value="220" unit="mA"/>
<define name="MILLIAMP_AT_FULL_THROTTLE" value="4900" unit="mA"/>
<define name="MILLIAMP_AT_FULL_THROTTLE_VOLTAGE" value="8.0" unit="V"/>
```

Paparazzi then adjusts only the estimated motor-current portion for live battery voltage. If the voltage reading is unavailable or invalid, it safely uses the configured current without voltage adjustment.

This fallback is useful, but a calibrated real current sensor remains the better source: it sees the actual motor, servos, avionics, wiring losses, and whatever else is consuming electrons today.
