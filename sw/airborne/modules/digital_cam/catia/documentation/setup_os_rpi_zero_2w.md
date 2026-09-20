# Prepare a Pi Zero 2 W for CATIA

Use the official [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to
write Raspberry Pi OS to a microSD card. Use a current 64-bit Raspberry Pi OS
image for MORA. Do not replace or rename `/usr/bin/rpi-imager`; package-managed
launchers should remain under package-manager control.

Before flashing, make sure the selected Wi-Fi network is reachable by a Raspberry
Pi Zero 2 W. It supports 2.4 GHz Wi-Fi; use WPA2/WPA3 personal security, not
enterprise authentication or TKIP.

1. In Imager, select **Raspberry Pi Zero 2 W**, a current 64-bit Raspberry Pi
   OS image, and the intended microSD card. Confirm its capacity and name before
   continuing: writing erases the selected card.
1. Open OS customization. Set a hostname, a non-default user, the 2.4 GHz Wi-Fi
   network and country, then enable SSH. Prefer public-key authentication; paste
   the public key from the development PC into **Authorized keys**.
1. Write the card and wait for Imager's verification to succeed. Eject it,
   insert it into MORA, and power the board.
1. From the development PC, confirm the configured account and network work:

   ```sh
   ssh air@theatre
   ```

For Imager-specific screens and recovery options, see the official
[Raspberry Pi getting-started documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html).

## Configure UART for CATIA

On MORA, disable Bluetooth and dedicate the full UART to CATIA. Run the following
on MORA, then reboot:

```sh
sudo nano /boot/firmware/config.txt
sudo raspi-config
sudo reboot
```

Under the existing `[all]` section in `config.txt`, add:

```ini
enable_uart=1
dtoverlay=disable-bt
```

In `raspi-config`, select **Interface Options > Serial Port**, answer **No** to
the login shell, and **Yes** to serial hardware. After reboot, verify the full
UART mapping and user access:

```sh
ls -l /dev/serial0 /dev/ttyAMA0
id -nG
```

Expected mapping: `/dev/serial0 -> ttyAMA0`. The account running CATIA must be a
member of `dialout`; add it with `sudo usermod -aG dialout air`, then log out and
back in.

## Preventing Sleep and Suspend States

Raspberry Pi OS normally does not suspend a headless Pi. If a board is managed
by software that might request suspend, disable those targets explicitly:

```sh
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

This does not overclock the board and is not a substitute for stable power and
thermal management. Revert it with `sudo systemctl unmask sleep.target
suspend.target hibernate.target hybrid-sleep.target` if suspend is later needed.
