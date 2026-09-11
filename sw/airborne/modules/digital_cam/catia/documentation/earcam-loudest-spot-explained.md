# EARcam Explained: Finding the Loudest Spot With One Simple Microphone

This document is for readers who want to understand, without a signal
processing background, what is involved when a small aircraft tries to find
the loudest spot on the ground. It explains why the EARcam solution was built
the way it is, how far a plain USB microphone can realistically hear an MSA
motionSCOUT alarm, and step by step what the software does to turn thousands
of noisy sound readings into one position on the map.

The companion [README](README.md) describes how to build, run and test the
system. This document explains the reasoning behind it.

## The task in one sentence

A person wearing an MSA motionSCOUT personal alarm has stopped moving. The
alarm sounds. An aircraft flying overhead must work out where the sound comes
from, mark that spot as a waypoint and fly a drop run over it.

Nothing on the aircraft points a "sound camera" at the ground. There is one
ordinary USB microphone. The trick is that the aircraft moves: every tenth of a
second the software knows exactly where the microphone was, how high it was and
how loud the alarm sounded there. Thousands of such geotagged readings,
gathered along a planned flight path, together behave like one very large
microphone array spread over the whole search area. Software then asks a
simple question of all of them: "which point on the ground explains these
loudness readings best?"

## Why one simple USB microphone was chosen

Several ways exist to locate a sound source. The table lists the main ones and
why they were not used here.

| Approach | What it needs | Why it was not chosen |
| --- | --- | --- |
| Microphone array on the aircraft (direction finding by time differences) | Several microphones, all sampled at exactly the same instant, spaced about half a wavelength apart (5 to 8 cm at 2 to 3 kHz), mounted away from propeller and airflow noise | Multi-channel synchronized audio hardware, calibration of every channel, sensitivity to wind and motor noise, extra weight and power on a 250 g class aircraft |
| Directional (shotgun or parabolic) microphone | A microphone that hears mostly in one direction, plus a way to sweep it | Bulky, wind sensitive, and a fixed-wing aircraft cannot hover and sweep |
| Ground sensor network | Sensors placed in the area before the alarm | Not possible in a search scenario |
| Single omnidirectional microphone plus aircraft motion (chosen) | One USB microphone, GPS and attitude from the autopilot, a planned flight path | Cheap, light, no calibration between channels, robust; the aircraft's own movement provides the "many positions" that an array would otherwise need |

The chosen approach trades hardware complexity for flight time and software.
It cannot tell direction from a single reading, but it does not need to: over
a one minute survey it gathers more than a thousand readings from more than a
thousand different places. That is the key idea of the whole solution.

A second reason is practical robustness. A single microphone has one failure
mode (present or not present). The software is written so that a missing or
dying microphone never stops the rest of the camera pipeline: the reading is
simply marked invalid and the microphone process is restarted every three
seconds until it works again.

## How sound behaves, in plain terms

### Decibels

Loudness is measured in decibels (dB). The scale is logarithmic, which means
that equal steps in dB are equal ratios, not equal amounts:

- +6 dB means the sound pressure doubled.
- +10 dB is roughly "twice as loud" to the ear.
- +20 dB means ten times the sound pressure.

Two flavours of dB appear in this system:

- **dB SPL** (sound pressure level) is the physical loudness in the air. The
  motionSCOUT manual specifies at least 95 dB SPL at 3 m.
- **dBFS** (decibels relative to full scale) is what a digital microphone
  reports: 0 dBFS is the loudest value the converter can represent, and
  everything real is negative, for example -40 dBFS. Converting dBFS to
  dB SPL needs a calibration offset that is different for every microphone,
  gain setting and mounting. EARcam does not need that offset to find the
  loudest spot, because it only compares readings with each other. It only
  needs it for the rough distance estimate.

### The inverse distance law

In open air, sound from a small source gets 6 dB quieter every time the
distance doubles. Starting from 95 dB at 3 m:

| Distance to alarm | Expected level (free field) |
| --- | --- |
| 3 m | 95 dB SPL |
| 6 m | 89 dB SPL |
| 12 m | 83 dB SPL |
| 25 m | 77 dB SPL |
| 50 m | 71 dB SPL |
| 100 m | 65 dB SPL |
| 200 m | 59 dB SPL |
| 400 m | 53 dB SPL |

Real air also absorbs high frequencies: at 2 to 3 kHz about 1 dB extra is lost
per 100 m, more in dry cold air. Ground reflections can add or remove a few dB
depending on where the alarm lies, and a person lying on the alarm can block
it partly. These effects make an absolute distance estimate rough, but they do
not change where the loudest spot is: the alarm is still loudest when the
aircraft is directly above it.

### The "flat top" problem

The aircraft flies at a height. Suppose it flies at 40 m above ground. When it
is directly above the alarm the distance is 40 m. When it is 10 m to the side,
the straight-line distance is only 41.2 m (Pythagoras). The level difference is
just 0.26 dB, far smaller than the natural wobble of any real measurement.

| Height above ground | 10 m to the side | 20 m to the side | 40 m to the side |
| --- | --- | --- | --- |
| 20 m | 1.0 dB quieter | 3.0 dB quieter | 7.0 dB quieter |
| 40 m | 0.3 dB quieter | 1.0 dB quieter | 3.0 dB quieter |
| 60 m | 0.1 dB quieter | 0.5 dB quieter | 1.6 dB quieter |

This is the single most important limitation of the method. The loudness
"hill" seen from the air has a flat top whose width is about the flight
height. Two consequences follow:

1. The lower the aircraft flies, the sharper the peak and the more precise the
   result. This is why the coarse survey flies at 60 m and the refinement at
   40 m above ground.
2. A single straight pass can only say where along the track the peak was. It
   cannot tell whether the alarm was 15 m left or 15 m right of the track,
   because both give nearly the same level. Several passes from different
   directions are required. That is why the refinement is flown as a star.

## What a simple USB microphone can and cannot do

The microphone used on MORA is a generic USB dongle with a Texas Instruments
PCM2902 codec (the same chip is inside most look-alike dongles from other
brands, and the software accepts any USB audio device with a recording input).
It delivers mono 16-bit audio at 48 000 samples per second.

What was measured and configured:

- **Gain is fixed** at mixer step 14 of 16 (about +21 dB) and **automatic gain
  control is switched off**. This matters more than anything else: if the
  microphone were allowed to adjust its own gain, a louder reading would no
  longer mean a closer alarm.
- **Noise floor** in the alarm band with the microphone idle is about
  -90 dBFS. Readings below this are treated as "nothing heard".
- **Clipping** starts near the digital limit (32700 of 32767). A clipped
  reading is flagged and excluded, because a distorted signal no longer
  measures loudness.
- **Usable dynamic range** is therefore roughly 80 dB between "nothing" and
  "too loud". Because the alarm level falls only 6 dB per doubling of
  distance, this range covers everything from a few metres to well beyond the
  distance at which the alarm disappears into the aircraft's own noise.

What limits the detection distance in practice is not the microphone's
sensitivity but the noise around it:

- **Propeller, motor and airflow noise** on the aircraft are typically the
  loudest things the microphone hears. Airflow noise is mostly low frequency;
  propeller and motor harmonics can fall inside 1.8 to 3.2 kHz.
- **Wind on the ground** and vegetation add broadband noise.
- **The alarm pulses**: the motionSCOUT sounds three times per second, so about
  half the time there is no tone to hear at all.

The software addresses the noise with a tone detector (described below) that
compares the strongest single tone with everything else in the band. A
motionSCOUT tone is a sharp spike; wind and propeller noise are smeared across
the band. As a rule of thumb, with the alarm at 95 dB at 3 m and the aircraft
self-noise in the band at 60 to 70 dB SPL at the microphone, the alarm is
detectable out to roughly 50 to 150 m of straight-line distance. This is why
the survey pattern uses 25 m track spacing at 60 m height: every point of the
search area is within about 65 m of some sample. The exact detection range for
a given airframe must be measured in flight; the numbers above are estimates.

Finally, without an acoustic calibrator the **absolute distance** shown by the
`earcam` tool (from 95 dB at 3 m and the inverse distance law) is only a rough
indication. The loudest spot search does not use it. It only ever compares
readings taken by the same microphone during the same flight.

## The measurement chain, step by step

### 1. From air pressure to numbers

The microphone converts pressure into voltage, the PCM2902 converts voltage into
16-bit numbers 48 000 times per second, and Linux ALSA delivers them to the
`earcam` program.

### 2. Fifty millisecond windows

`earcam` cuts the stream into windows of 50 ms (2400 samples). Fifty
milliseconds is short enough to follow a 12 m/s aircraft to within 0.6 m and
long enough to resolve tones about 20 Hz apart.

### 3. Band-pass filter

Everything outside 1.8 to 3.2 kHz is removed. The motionSCOUT manual states a
2 to 3 kHz alarm; the margin covers filter roll-off, unit variation and Doppler
shift (at 12 m/s the pitch shifts by up to about 3.5 percent, close to 90 Hz
at 2.5 kHz). Removing the rest of the spectrum discards most airflow and engine
noise before it can influence anything.

### 4. Fifty-seven tone detectors

Inside the band, 57 narrow tone detectors (Goertzel filters, each tuned 25 Hz
from its neighbour) measure how much energy sits at each frequency. A Hann
window is applied first so that a tone between two detectors does not leak
into all the others. The detector with the most energy gives:

- **level**: the strength of the strongest tone, in dBFS;
- **frequency**: which tone it was;
- **contrast**: the strongest tone compared with the total in-band energy. A
  pure alarm tone gives a contrast near 0 dB; broadband noise gives a strongly
  negative contrast. A window counts as "alarm present" when the contrast is
  better than -3 dB and the level is above -90 dBFS.

### 5. Bridging the pulses

The motionSCOUT alarm pulses three times per second. An envelope follower holds
the last level and lets it decay at 6 dB per second, so the gaps between pulses
do not look like the aircraft flying away. A fast and a slow envelope are
compared to label each window as approaching (`env` rising by more than
0.8 dB), receding or steady. This trend is used later as a small bonus when
choosing the most trustworthy samples.

### 6. One text line every 50 ms

`earcam --server` prints one line per window containing time, level,
frequency, contrast, trend, envelope, alarm flag, clip flag and calibration.
The CATIA camera service reads these lines from a pipe. There is nothing
acoustic left to do on the aircraft after this point; everything else is
geometry and statistics.

## Geotagging: attaching a place to every reading

The Paparazzi autopilot treats EARcam like a camera. During a search it sends a
"targeted shoot" message to CATIA on the MORA computer every 100 ms containing the
aircraft's latitude, longitude, altitude, height above ground, heading, roll
and pitch, exactly as it would for a photo. CATIA pairs each message with the
newest microphone reading and stores the pair. Up to 16 384 such samples fit
in memory, about 27 minutes of continuous listening.

At 12 m/s and 10 samples per second, there is one geotagged reading every 1.2 m
along the flight path. The autopilot only counts every tenth sample as a real
"photo" for the ground station, so the 9600 baud telemetry link is not flooded.

## The flight pattern

The pattern is a compromise between covering the area, keeping the aircraft
safe and getting the flat top problem under control.

1. Start listening.
2. Lawn-mower survey of the search polygon at 60 m above ground, 25 m
   between tracks.
3. Coarse solve (listening continues).
4. Star refinement at 40 m above ground: four straight legs of 100 m through
   the estimate.
5. Solve again. If the answer moved less than 4 m, or three stars have been
   flown, continue; otherwise fly another star around the new estimate.
6. Stop listening: final solve over all samples and render the sound picture.
7. Move the `DROP` waypoint to the answer and place the run-in waypoint three
   turn radii before it.
8. Fly the drop run.

The data flow between autopilot, CATIA and `earcam` is drawn in
[earcam-dataflow.png](earcam-dataflow.png).

### Coarse survey

A lawn-mower pattern (Paparazzi's polygon survey) covers the search polygon at
60 m above ground with 25 m between tracks. The purpose is to make sure the
alarm is heard at all and to get a first estimate that is good to within a few
tens of metres. The survey stops as soon as the polygon has been swept once.

### Star refinement

The autopilot asks CATIA for an interim result while the microphone keeps
listening. It then lays out a star of four straight legs, each 100 m long,
all passing through the estimated spot, at 40 m above ground. Each leg
crosses the loudness hill from a different direction, so the across-track
ambiguity of one leg is resolved by the next.

The turns between legs are true fillet turns: the aircraft flies a circular arc
that is tangent to both the leg it leaves and the leg it enters. The leg half
length is enlarged automatically when the turn radius would otherwise not fit
(half length = max(50 m, R / tan(90 degrees / N) for N legs). This keeps the
aircraft on the straight legs when it matters and avoids wasted S-turns.

After the star another solve is requested. If the new answer moved less than
4 m from the previous one the search has converged; otherwise a new star is
flown around the new estimate, at most three times. The star can be started by
hand from the ground station as well.

### Going to the spot

Listening is stopped, CATIA computes the final answer from every sample and
renders the sound picture. The autopilot moves the `DROP` waypoint to the
result and places a run-in waypoint three turn radii before it, aligned with
the last leg, so the drop approach is a stable straight line. If nothing was
heard, the survey is flown once more; if it still fails, the aircraft goes to
standby instead of dropping blindly.

## Turning readings into one position

This is what happens inside CATIA on MORA each time the autopilot asks for a result.
Every step exists to defeat a specific way in which real data misleads.

### Step 1: gate the samples

Samples are thrown out if the microphone reported clipping, if the level was
below the -90 dBFS floor, or if the position was invalid. If at least eight
samples carried the "alarm present" flag, only those are used; otherwise all
remaining samples are used so that a weak, distant alarm still yields an
answer with low confidence.

### Step 2: reject outliers

A gust, a bird or a radio click can produce one very loud or very quiet
window. A Hampel filter computes the median level and the median absolute
deviation (MAD), a robust measure of spread, and removes samples that lie far
below the bulk. Unlike an average and standard deviation, these two measures
are not themselves dragged around by the very outliers they are meant to
catch.

### Step 3: work in metres, not degrees

Latitude and longitude are converted into east and north metres around the
loudest sample. All geometry from here on is done in this flat local frame,
which is accurate to millimetres over a few hundred metres.

### Step 4: a first guess from the loudest quarter

The loudest 25 percent of the samples are selected. Their positions are
averaged, weighting each by its acoustic power (10^(level/10)) so a sample
3 dB louder counts twice as much, with a small extra weight for samples that
were labelled "approaching". The result is a robust first guess that already
lies inside the flat top of the hill.

### Step 5: fit the physics

The first guess is refined by fitting the inverse distance law to all the
kept samples. For a candidate ground position (x, y) the expected level at
sample i is

    L_i = L0 - 10 log10( d_i^2 + h_i^2 )

where d_i is the horizontal distance from the sample to (x, y), h_i is that
sample's own height above ground, and L0 is the unknown source strength. The
per-sample height is essential: the survey at 60 m and the star at 40 m give
different levels for the same ground distance, and using each sample's real
height makes them comparable.

A damped Gauss-Newton iteration adjusts x, y and L0 to make the predicted
levels match the measured ones as closely as possible. Three safeguards make
this robust:

- **Huber weights** reduce the influence of any sample whose residual is much
  larger than the rest, so a few bad readings cannot pull the answer.
- **Step bound** of 25 m per iteration prevents the fit from jumping out of
  the area where the linearisation is valid.
- **Drift guard**: if the fit wanders more than 60 m from the first guess, or
  fails to converge, the first guess is returned instead.

Using the physics model rather than only the weighted average matters most
when the survey lines were not symmetric around the alarm: a plain average is
pulled towards wherever more samples happened to be taken; the fit is not.

### Step 6: how sure are we?

A confidence value between 0 and 1000 is reported with every result. It is
built from three parts:

- **Prominence (50 percent)**: how much louder the peak is than the typical
  sample. A sharp, loud peak is trustworthy; a flat field of near-equal levels
  is not.
- **Concentration (30 percent)**: how tightly the loudest quarter of the
  samples clusters around the answer.
- **Support (20 percent)**: how many samples carried the alarm flag.

The confidence is informative; the autopilot's decision to accept a result is
based on it being valid and on the star iterations converging.

### Step 7: the answer

CATIA replies with latitude, longitude, the ground altitude under the spot, the
median height above ground of the loudest samples, the peak level, the
confidence and the number of samples used. The autopilot keeps the last valid
answer if a later request returns invalid, so a moment of silence at the wrong
time cannot erase a good result.

## The sound picture

Besides the position, CATIA renders a JPEG "sound picture" so a human can judge
the result at a glance and so the image can flow through the same photo
pipeline as an ordinary camera picture (EXIF geotag, SODA analysis).

![Example sound picture from the simulator](earcam-sound-picture-example.jpg)

Each sample deposits its acoustic power on a grid of cells (cell size a
quarter of the mean flight height, at least 2 m) with a Gaussian footprint
whose width is half the flight height, mirroring the flat top described
above. The colours span the top 20 dB from blue through cyan and yellow to
red. Grey dots show where the aircraft was when each sample was taken, the
white ring marks the computed loudest spot, and a 50 m scale bar gives the
size. The long side of the image is always 768 pixels.

## Applied to IMAV 2026 Mission 4

The competition rules (rulebook 5.4.7) turn the general problem into a very
specific one, and the flight plan `easystar3_imav2026_mission4_earcam.xml`
is shaped by them:

| Rule | Consequence for the design |
| --- | --- |
| The alarm mannequin lies within 25 m of a GPS point given on the day | No survey. The first star is flown straight around the given point; any result more than 35 m from it is rejected as noise. |
| Three mannequins, only one with the alarm | The two silent ones are invisible to the microphone; the loudest spot is the right one by construction. |
| Points only for a release below 2 m, as judged from the ground | The run-in descends to 1.5 m on the rangefinder, and the hatch will not open above 2 m. Better no drop than a drop that scores nothing. |
| Full points within 50 cm of the navel, none beyond 300 cm | Everything is about the last metres: low star (sharp peak), slow flight (10 m/s), release timed by `nav_drop` from speed, wind and measured height. |
| Score doubled on the alarm mannequin, weight factor up to 2 | A light EasyStar 3 with everything on board (autonomy factor 1.0) can reach the maximum. |
| The motionSCOUT K-T-R sounds at 2.6 to 3.0 kHz | The tone search is narrowed to 2.4 to 3.2 kHz, rejecting more propeller noise. |

Two further facts about the aircraft shape the plan. First, an EasyStar 3
has a pusher propeller a hand's width from the microphone: with the motor
running the alarm is buried. So every star leg cuts the throttle 40 m before
the estimate and restores it 30 m past it; the aircraft glides from 12 m
to about 6 m over the spot and only the samples taken after the propeller
has stopped (1.5 s) count. Second, it glides well and flies slowly, so a
release at 10 m/s from 1.5 m means the kit travels only about 5 m after
leaving the aircraft, and a 0.1 s timing error costs 1 m. That is why the
release is commanded in the very navigation cycle the release point is
crossed, and why the hatch delay and the GPS position latency must be
measured and entered as `TRIGGER_DELAY`.

In the simulator the complete Mission 4 loop, with the alarm 9 m from the
given point, took two stars (about 4.5 minutes), placed the loudest spot
0.8 m from the alarm, released at 1.50 m and put the kit about 2.3 m from
it; the remaining error is the simulated GPS latency along track and the
route-following error across track, both of which are tuning items on the
real aircraft.

## What accuracy to expect

In the simulator, with an ideal inverse distance source and 1.5 dB of random
microphone noise, the complete loop (54 s survey, one four-leg star, 1382
samples) placed the `DROP` waypoint 0.8 m from the true alarm position.

In the real world the following will add error, roughly in order of
importance:

1. **Flight height**: the flat top scales with height. Refining lower than
   40 m helps directly, within the limits of safety.
2. **Ground reflections and obstructions**: a body over the alarm, a ditch or
   a wall can shift the apparent peak by several metres.
3. **Aircraft self-noise** varying with throttle and bank angle: a turn at
   full power looks quieter than a glide. The contrast test and the outlier
   rejection reduce this; flying the star legs at constant throttle reduces it
   further.
4. **GPS position error** (typically 1 to 3 m) is added directly to the answer.
5. **Timing**: the microphone reading and the position message are paired to
   within 50 ms, about 0.6 m at 12 m/s.

An overall real-world accuracy of 5 to 15 m is a realistic expectation for a
first deployment; the drop run-in geometry and the star convergence test are
designed with this in mind.

## Ways to do better later

- **Fly lower on the last star** once confidence is high.
- **Calibrate the microphone** with an acoustic calibrator so that the
  distance estimate becomes meaningful and can be used as an extra constraint.
- **Mount the microphone** behind a windscreen, away from the propeller, and
  measure the self-noise spectrum of the airframe so the band can be tuned.
- **Fuse repeated stars** flown at different heights, which the per-sample
  height model already supports.
- **A two-microphone stereo pair** would add a left/right cue; the fusion
  would then receive a direction in addition to a level.

## Glossary

- **AGL**: above ground level; the aircraft's height over the terrain, not over
  sea level.
- **ALSA**: the Linux sound system that delivers microphone samples to
  programs.
- **Band-pass filter**: keeps frequencies within a range and removes the
  rest.
- **CATIA**: the camera service on the MORA computer that talks to the
  autopilot and to the cameras, here including EARcam.
- **Contrast**: the strongest tone compared with all in-band energy; high for a
  clean alarm tone, low for noise.
- **dBFS / dB SPL**: see "Decibels" above.
- **Envelope**: a smoothed loudness that ignores short gaps.
- **Fillet turn**: a circular arc tangent to two straight legs.
- **Gauss-Newton**: an iterative method to find the parameters of a model that
  best explain measurements.
- **Goertzel filter**: an efficient way to measure the energy at one chosen
  frequency.
- **Hampel filter / MAD**: robust outlier rejection based on medians.
- **Huber weights**: reduce the influence of samples that disagree strongly
  with the rest.
- **MORA**: the Raspberry Pi companion computer on the aircraft.
- **motionSCOUT**: MSA's personal motion alarm; sounds at 95 dB at 3 m,
  2 to 3 kHz, three pulses per second.
- **NPS**: Paparazzi's flight simulator, used to test the complete loop
  without flying.
- **Sound picture**: the JPEG heat map produced at the end of a search.
