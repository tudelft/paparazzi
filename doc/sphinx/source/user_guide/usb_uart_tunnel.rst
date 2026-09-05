USB to UART Tunnel
==================

The USB to UART tunnel lets a program on your computer communicate directly
with a serial device connected to the flight controller. The flight controller
simply forwards bytes between its USB port and the selected UART in both
directions.

This is useful when a device is already installed in an aircraft and is
difficult to unplug. For example, you can connect u-center to an installed
u-blox receiver to:

* inspect its current configuration;
* change and save receiver settings;
* watch live UBX messages;
* use the UBX-MON-SPAN spectrum analyzer while testing the installation.

The tunnel does not interpret or alter the serial protocol. It can therefore
also be used with other UART peripherals and their normal configuration tools.

Choose how to run the tunnel
----------------------------

There are two ways to use it:

* The dedicated ``usb_tunnel`` firmware runs only the tunnel. This is the
  simplest and safest choice for normal configuration work.
* The ``usb_uart_tunnel`` module runs inside the main autopilot firmware. This
  is useful when other onboard equipment must remain active during a bench
  test, for example while investigating electrical or radio-frequency noise.

The selected USB port and hardware UART must be dedicated to the tunnel. Two
modules cannot read the same serial data reliably.

Normal GPS configuration
------------------------

During normal operation, the u-blox driver owns the GPS UART and provides GPS
data to the autopilot. A typical configuration is::

  <module name="gps" type="ublox">
    <configure name="GPS_BAUD" value="B460800"/>
    <configure name="GPS_PORT" value="UART4"/>
  </module>

Use this configuration for flight. Do not enable the tunnel on UART4 at the
same time.

Temporary tunnel in the autopilot
---------------------------------

For bench testing, replace the normal UART GPS module above with the tunnel
module and a datalink GPS provider::

  <!-- Bench-only configuration: UART4 is a raw USB tunnel, so this
       firmware has no GPS input and must not be used for flight. -->
  <module name="usb_uart_tunnel">
    <configure name="TUNNEL_PORT" value="UART4"/>
    <configure name="TUNNEL_BAUD" value="B460800"/>
  </module>

  <!-- Satisfy INS/navigation without assigning a hardware UART to GPS. -->
  <module name="gps" type="datalink"/>

The ``gps_datalink`` module keeps the autopilot's GPS interfaces available for
the build, but it does not read UART4. Unless an external system supplies GPS
data over the datalink, the autopilot has no GPS fix in this mode.

If telemetry normally uses USB, move it to a separate physical UART so the
tunnel has exclusive use of ``usb_serial``. For example::

  <module name="telemetry" type="transparent">
    <configure name="MODEM_PORT" value="UART3"/>
    <configure name="MODEM_BAUD" value="B57600"/>
  </module>

This arrangement keeps the main autopilot and UART3 telemetry running while
USB is connected directly to the device on UART4.

.. warning::

   An autopilot image using ``gps_datalink`` instead of the hardware GPS driver
   is for bench use only. Remove the tunnel module, restore the normal GPS
   module, rebuild, and upload the flight configuration before flying.

Dedicated setup firmware
------------------------

The dedicated ``usb_tunnel`` target is the recommended option when the only
goal is to inspect or configure a connected serial device. It temporarily
replaces the normal autopilot application with a small firmware whose main job
is forwarding bytes between USB CDC and one hardware UART.

This approach has several practical advantages:

* the selected UART and USB connection cannot conflict with telemetry or a
  device driver in the autopilot;
* flight-control and navigation modules do not compete with a busy serial
  stream;
* configuration tools see a transparent connection to the installed device;
* the normal flight configuration remains in the airframe XML and can be
  restored simply by uploading the ``ap`` target again.

Add a separate ``setup`` firmware block to the airframe. It does not replace or
modify the existing ``fixedwing`` or ``rotorcraft`` configuration::

  <firmware name="setup">
    <target name="usb_tunnel" board="matek_f405_te_sd">
      <configure name="TUNNEL_PORT" value="UART4"/>
      <configure name="TUNNEL_BAUD" value="B460800"/>
    </target>
  </firmware>

The two settings describe the device-facing side of the tunnel:

``TUNNEL_PORT``
  The flight-controller UART connected to the device. For this example the
  u-blox receiver is connected to UART4.

``TUNNEL_BAUD``
  The baud rate used on that hardware UART. It must match the rate already
  configured in the connected device; this example uses 460800 baud.

``TUNNEL_PORT`` defaults to ``GPS_PORT`` and ``TUNNEL_BAUD`` defaults to
``B115200``. Setting a baud rate in the host application does not reconfigure
the board UART; it must match the compiled ``TUNNEL_BAUD`` value.

When this target is installed, the flight controller is acting as a USB-to-UART
adapter. The normal autopilot, telemetry schedule, GPS parser, navigation, and
flight-control functions are not running. The tunnel starts automatically
after boot and remains active until another firmware image is uploaded.

In Paparazzi Center:

#. Select the aircraft containing the setup block.

#. Select the ``usb_tunnel`` target instead of ``ap``.

#. Build and upload the target. If the uploader waits for the bootloader,
   power-cycle the flight controller once.

#. Wait for the board to reappear as a USB serial device.

#. Open that serial device in the peripheral's configuration program, using the
   baud rate configured by ``TUNNEL_BAUD``.

Only one host program can own the USB serial device. Close Paparazzi serial
tools, ``socat``, terminal programs, and any previous configuration-tool
connection before opening it elsewhere.

After completing the device setup, close the host program and upload the
aircraft's normal ``ap`` target. Power-cycling alone does not restore the
autopilot because the setup firmware remains stored in flash.

.. warning::

   The dedicated setup firmware is not flight firmware. Keep the aircraft
   disarmed and remove the propeller while it is installed. Always upload and
   verify the normal ``ap`` target before flight.

Build and upload
----------------

For the dedicated firmware, select the aircraft and ``usb_tunnel`` target in
Paparazzi Center, then build and upload it. The equivalent command is::

  make AIRCRAFT=<aircraft> usb_tunnel.upload

For the integrated module, build and upload the normal ``ap`` target instead.
An autopilot bootloader may require one USB power cycle while the uploader is
waiting.

Where possible, configure the board upload rule to select the bootloader by its
persistent ``/dev/serial/by-id/`` identity. Avoid broad ``/dev/ttyACM*``
patterns on computers that also contain USB or LTE modems.

Wiring
------

Serial data lines must be crossed:

* Peripheral TX to flight-controller RX
* Peripheral RX to flight-controller TX
* Ground to ground

Power the peripheral according to its voltage requirements. The flight
controller normally continues to power a permanently installed receiver while
the tunnel is active. Do not connect an additional USB-to-UART adapter power
pin when either device is already powered.

Check the connection on Linux
-----------------------------

On Linux, prefer a stable udev symlink for the Paparazzi CDC device. A quick
receive-only check is::

  timeout 10s socat -u -x -v \
    FILE:/dev/paparazzi/stm32-usb-serial,b460800,raw,echo=0 STDOUT

Receiving hexadecimal data confirms the peripheral-to-computer direction. Stop
``socat`` before opening another program because only one application should
own the serial port at a time.

Using u-center through Wine
---------------------------

Map an unused Wine COM port to the stable Paparazzi link. This example uses
COM34::

  ln -sfn /dev/paparazzi/stm32-usb-serial \
    "$WINEPREFIX/dosdevices/com34"

Then:

#. Start u-center in the same Wine prefix.
#. Select ``COM34`` as the receiver connection.
#. Select the baud rate compiled into the tunnel, for example ``460800``.
#. Confirm that the connection indicator is active and receiver data updates.

The Linux USB CDC baud setting does not change the flight-controller UART. The
``TUNNEL_BAUD`` value in the aircraft configuration remains authoritative.

Use it to fix GNSS interference source
--------------------------------------

This method is especially helpful because the receiver remains powered and
wired exactly as installed. It can reveal interference caused by mounting
position, nearby digital electronics, power wiring, or transmitters that may
not appear when the receiver is tested separately on a desk.

A supported u-blox M10 can report a live radio spectrum through the
``UBX-MON-SPAN`` message. In u-center, open **UBX-MON (Monitor) -> SPAN
(Spectrum Analyzer)**. Keep the aircraft stationary and use the live display
as a comparative tool:

#. Observe the baseline with optional avionics and transmitters switched off.

#. Switch one device on at a time, such as an ESC, camera computer, telemetry
   radio, or switching regulator.

#. Look for a raised noise floor or new narrow peaks in the GNSS frequency
   bands.

#. Move the GNSS receiver, antenna, or noisy cable to another realistic mounting
   position and compare the display again.

#. Repeat with the final cable routing and all normal onboard equipment active.

Avoid changing receiver settings while moving hardware. First compare the
spectrum consistently, then make one installation change at a time. Keep the
propeller removed and the aircraft disarmed whenever the main autopilot is
running on the bench.

Return to flight configuration
------------------------------

After configuration, restore the hardware peripheral module and upload the
normal flight configuration before flight. For the GPS example, remove
``usb_uart_tunnel`` and ``gps_datalink``, then restore the original
``gps``/``ublox`` block shown above.
