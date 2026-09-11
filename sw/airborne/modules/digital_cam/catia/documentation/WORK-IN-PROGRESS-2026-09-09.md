# Offline Checkpoint: 2026-09-09

## Superseded On 2026-09-10

This file preserves the earlier pause state; everything below this section is
historical, not a current task list. Callback-based capture, the SDK duplicate
polling regression, bounded pose logging, schema-2 diagnostic clock bounds and
serialized partial-write-safe UART output are now implemented. Native capture,
callback, raw/EXIF, sender, logger and clock/UART regressions and the current
ARM64 build passed on 2026-09-10. Existing vendor build warnings remain.

The user has requested the minimum first test: **freeze feature work** and use
the [first-test checklist](mission2-score-first.html#first-test-checklist).
Streaming, logging and clock probes remain opt-in; no aircraft configuration,
deployment or physical camera/flight test was performed. Actual Talon firmware,
UART capacity, MORA execution and recovered camera data still need bench checks.
Clock bounds are logged, not applied to geolocation. Measured calibration and
independent target references are required for accuracy assessment. Interpolation,
effective-lag estimation, fusion, replanning and final export are deferred.
Do not restore the old backup over current sources.

## Resume Here

Paused at the user's request before going offline. Resume the Proposed Plan's
accuracy work, not another planning-only rewrite. The active slice is a
timestamped FC pose evidence stream for later camera/pose alignment. It is
not complete, enabled, deployed, or flight validated.

The current files are saved in the working trees. A separate local source
snapshot is being created outside the workspace; its final path and verification
are recorded in repository memory and the assistant's close-out message.
Do not restore it over the working trees automatically: they contain unrelated
user changes, including staged files. No commit, stash, branch change, or reset
was requested or performed.

## Requirements To Preserve

- Maximize Mission 2 points with the existing Talon, Tiny1-C, M10N and MORA.
  Hardware replacement is not an option.
- Software, calibration and timing first. Change flight behaviour only if there
  is no good alternative, preserving clearance, control margins and landing reserve.
- User's concern about airframe loss is incomplete recovery of Mission 2 data,
  not automatic loss of all results or all competition opportunities.
- Mission 3 uses a separate quadcopter; Mission 4 uses EasyStar. Talon loss does
  not remove those aircraft's availability. Do not infer a Mission 1 assignment.
- Land after Mission 2 to access the SD card. The five-minute submission clock
  includes recovery, shutdown, card access, copying and handover. No live result
  access is assumed. Retain useful partial results.
- Do not modify bundled libexif or require ExifTool in production. Do not add
  memcpy calls. Use bounded copy loops and existing project conventions.
- Keep raw Tiny1-C combined frame bytes beside the ordinary JPEG as .jpg.raw.
  Do not invent another radiometric JPEG format for new real captures.

## Pending Slice

Paths below are relative to sw/airborne/modules/digital_cam unless noted.

1. catia/protocol.h adds CATIA_POSE_SAMPLE message ID 8 with a 100-byte payload
   and 105-byte complete wire frame. Existing messages are unchanged.
2. Its data fields are sequence, sample_begin_us, sample_end_us, the existing
   40-byte shot pose, NED velocity in existing SPEED_BFP units, GPS time-of-week
   in ms, week, hacc/vacc in cm, sacc in cm/s, fix, satellite count, valid_fields
   and flags. CATIA_POSE_SAMPLE_GPS_PRESENT denotes cached GPS fields being present,
   not a fresh fix or synchronized pose.
3. uart_cam_ctrl.c adds a DIGITAL_CAM_UART_POSE_STREAM compile-time flag defaulting
   to 0. No airframe or module config enables it. With the flag enabled, the
   existing periodic function sends one sample after dc_periodic(). The module
   XML schedules this at 10 Hz. The sequence advances even when UART space is
   unavailable so gaps remain visible; photo numbering is not advanced by a sample.
4. The sender checks uart_check_free_space(&(CAMERA_LINK), NULL, 105), timestamps
   around the state read with get_sys_time_usec(), and uses GPS fields only under
   USE_GPS. get_sys_time_usec() is uint32_t and wraps in approximately 71.6 minutes.
5. Fixed the in-progress function guard before pausing: the whole send_pose_sample
   definition is inside #if DIGITAL_CAM_UART_POSE_STREAM. The original shot-function
   comment is back above send_shot_frame.
6. catia/tests/pose_protocol_test.c checks payload size, offsets, little-endian
   layout, signed-data round-trip, modular timestamp wrap, and transport length.
   The short-payload test observes the wrong length; consumer rejection must still
   be implemented and tested in the receiver.

## Checks At This Checkpoint

Passed on the development PC:

```sh
gcc -std=c11 -Wall -Wextra -Werror \
  sw/airborne/modules/digital_cam/catia/tests/pose_protocol_test.c \
  sw/airborne/modules/digital_cam/catia/protocol.c \
  -o /tmp/catia-pose-protocol-test
/tmp/catia-pose-protocol-test
/tmp/catia-calibration-venv/bin/python \
  sw/airborne/modules/digital_cam/catia/documentation/test_mission2_assess.py
```

The wire test passes; all 16 assessment tests pass, including SD-card deadline
semantics. The latest sender edit is only the opt-in guard correction, not a
completed embedded validation. CATIA native/ARM64 builds and integration tests
passed earlier for the previous timing/native-raw implementation; those results
must not be attributed to the new FC pose sender.

The editor currently cannot resolve FC include paths or generated/airframe.h
for uart_cam_ctrl.c. No full FC compile has been run for this slice. Protocol
and test diagnostics were clear. Do not change user editor settings merely to
hide this configuration limitation.

## Next Actions

1. Test the actual uart_cam_ctrl.c sender with a host harness or a suitable
   existing FC target. Cover flag off/on, GPS off/on, UART full, sequence gaps,
   timestamp wrap and unchanged shot-number behaviour. No new telemetry should
   appear with the flag off. Host stubs are not a replacement for a real FC build.
2. Check that the new sample cannot crowd out command traffic. At 10 Hz it uses
   about 1050 bytes/s, or 9.1 percent of a 115200-baud 8N1 link, with a single
   frame occupying about 9.1 ms. A free-space check is not a complete priority or
   worst-case scheduling proof; examine actual buffers, commands and rates.
3. Add a MORA consumer in catia/catia.c's handle_received_message(). Reject any
   payload length other than 100; validate/decode into a bounded structure.
   Record MORA monotonic reception timing honestly as reception, not exposure.
4. Implement opt-in CSV pose logging with a bounded queue and separate writer,
   avoiding SD I/O in the serial receive path. Capture sequence/drops, clock
   domains, flags, units and startup/reboot information. Record failures; do not
   let diagnostic logging block normal capture traffic. Handle flush/close and
   SD durability deliberately. This logger and its queue DO NOT EXIST yet.
5. Add exact round-trip, malformed-input, queue-full, write-error and shutdown
   tests; integrate build/lifecycle and document enabling without selecting an
   aircraft configuration or deploying it. Review whether additional GPS age
   metadata is required before freezing the diagnostic message.
6. Only after recording usable evidence, implement/validate clock mapping,
   pose history/interpolation and exposure timing. No current code synchronizes
   FC samples to camera exposures. Keep the existing estimated/unknown flags.
7. Later stages remain target tracking/fusion, durable final two-target export,
   independent reference validation and the rehearsed land/retrieve/submit chain.

Useful anchors: conf/modules/digital_cam_uart.xml (10 Hz task); uart_cam_ctrl.c
(FC state sender); catia/protocol.c (transport, accepts arbitrary message IDs);
catia/catia.c (serial dispatcher and shutdown); catia/Makefile (explicit CFILES);
sw/airborne/modules/gps/gps.h (quality units); sw/airborne/mcu_periph/sys_time.h
(clock API); sw/airborne/mcu_periph/uart.h (free-space API).

## Existing Work To Keep

- Normal live JPEG plus exact .jpg.raw companion, SDK raw_data_cut decoding and
  legacy mock-layer compatibility; numerical geolocation and EXIF annotations.
- Continuous rolling frame-stability gate, instead of restarting eight frames
  per request. Startup/recovery heuristic still requires hardware evidence.
- Optional `--lwir-motion-compensation`, default off: bounded constant-velocity
  correction over server request-to-UVC-return interval only, original coordinates
  retained. Not true exposure timing; attitude/altitude remain uncompensated.
- Illustrated calibration guide in Markdown/HTML/PDF and desktop lens/mount helper.
- Offline Mission 2 score/coordinate assessment and fixed-hardware mission plan.

No camera was operated, no flight performed, no real calibration measured, and
no five-metre field accuracy has been demonstrated.

## Offline Tools And Recovery

- Workspace: /home/n3yh3hnii/paparazzi. Preserve both the Paparazzi tree and the
  linked LWIRcam source tree. Do not assume a single Git repository captures both.
- Current branches observed: Paparazzi brandnewday_4_RLNC_mesh_tryout; LWIRcam
  add_detect_hotspot. These were not changed. Do not revert other work.
- /tmp/catia-calibration-venv currently has NumPy, OpenCV, Markdown-independent
  PDF tooling (Playwright, pypdf, PyMuPDF) and GeographicLib. /tmp is not durable
  across cleanup/reboot; system Python has Markdown but not GeographicLib/OpenCV.
- CLI test/build binaries in /tmp are disposable; sources are preserved. Use the
  pinned mission2-assessment-requirements.txt and helper instructions if an
  environment must be recreated. Do not assume network access after resuming.
- System ExifTool is installed; production does not need it. Clear any stale
  EXIFTOOL/PERL5LIB override when running desktop fixture tests.
- Durable repository memory: /memories/repo/lwircam-geolocation-resume-2026-09-09.md.
  The newest entry takes precedence over its older historical entries.

Resume request: "Continue from the 2026-09-09 offline checkpoint. Validate the
opt-in FC pose sender, then implement the bounded MORA pose logger."
