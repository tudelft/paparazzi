This guide summarises simple, effective steps to reduce power usage on a Raspberry Pi Zero 2 W. The focus is on headless and always-on use-cases.

Baseline Assumptions:

* Headless system (no HDMI display)
* Raspberry Pi OS Lite
* SSH access available

== Disabling Unused Hardware ==

=== HDMI ===
Disable HDMI output to save ~20–30 mA.

Add to <code>/boot/firmware/config.txt</code>:
<pre>
hdmi_blanking=2
hdmi_force_hotplug=0
hdmi_ignore_hotplug=1
</pre>

=== LEDs ===
Disable the activity and power LEDs if not required.

Add to <code>/boot/firmware/config.txt</code>:
<pre>
dtparam=act_led_trigger=none
dtparam=act_led_activelow=on
</pre>

== Reducing CPU Power ==

=== Limit CPU Frequency ===
Lower the maximum CPU clock to reduce both idle and load power.

Add to <code>/boot/firmware/config.txt</code> (minimum is 600MHz):
<pre>
arm_freq=600
</pre>

=== Limiting Active CPU Cores ===

All Raspberry Pi boards except the Zero(W) and original Raspberry Pi have quad-core CPUs. For many low-power workloads, the system can be limited to two or even one core, so reducing the peak power demand especially during boot.

Edit <code>/boot/firmware/cmdline.txt</code> (keep everything on a single line):

<pre>
maxcpus=2
</pre>

== Reducing Bluetooth and WiFi Power Consumption ==

=== Wi-Fi ===
To disable at boot, add to <code>/boot/firmware/config.txt</code>:
<pre>
dtoverlay=disable-wifi
</pre>

=== Bluetooth ===
To disable at boot, add to <code>/boot/firmware/config.txt</code>:
<pre>
dtoverlay=disable-bt
</pre>

== Tuning the Pi for <b>autostream</b> ==

Lo-tech <b>[https://www.lo-tech.co.uk/autostream/ autostream]</b> is a locally installed service that connects HiFi separates to AirPlay and AirPlay 2 speakers, as is designed around the Pi Zero 2W. To minimise peak and average power consumption in this application:

=== <code>/boot/firmware/cmdline.txt</code> ===

Append (keep everything on a single line):

<pre>
maxcpus=2
</pre>

=== <code>/boot/firmware/config.txt</code> ===

Replace the whole file with this:

<pre>
[all]
# Enable system watchdog
dtparam=watchdog=on

# Disable bluetooth
dtoverlay=disable-bt

# Disable on-board audio
dtparam=audio=off

# Disable camera and DSI display autodetection
camera_auto_detect=0
display_auto_detect=0

# Automatically load initramfs files, if found
auto_initramfs=1

# Run in 64-bit mode
arm_64bit=1

# Fully power down HDMI
hdmi_blanking=2
hdmi_force_hotplug=0
hdmi_ignore_hotplug=1
disable_overscan=1

# Minimum GPU RAM for headless
gpu_mem=16

# Underclock CPU
arm_freq=700
arm_boost=0
force_turbo=0

# Slight undervolt (safe range is usually -2 to -4)
over_voltage=-2

# Zero2Go PSU board parameters
# If you are not using the Zero2Go PSU board, comment these four lines
enable_uart=1
dtparam=i2c1=on
dtparam=i2c_arm=on
dtoverlay=miniuart-bt

</pre>

== Notes ==

* Changes to configuration files require a reboot to take affect
* Test stability if lowering CPU voltage
* If the Pi cannot boot after editing config.txt or cmdline.txt, those files on the SD card can be accessed directly using a Windows PC (or Mac) by connecting the micro-SD card via a USB adapter.

== See Also ==

*[https://www.lo-tech.co.uk/autostream/ autostream (product page)]
*[https://www.lo-tech.co.uk/add-airplay-to-a-vintage-cd-player/ Adding AirPlay to a Vintage CD Player (blog post)]
