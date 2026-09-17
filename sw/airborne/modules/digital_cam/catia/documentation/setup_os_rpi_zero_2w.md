#Getting you OS ready from scratch

First make sure you router is setu so it can accepp the connection request from RPI

WPA2 (NOT WPA2 with SHA256) and NO TKIP bit 

To flash Raspberry Pi OS onto a microSD card for a Raspberry Pi Zero 2 W with preconfigured credentials, network settings, and SSH keys, use the official **Raspberry Pi Imager** software.
You can download it here Download https://www.raspberrypi.com/software/ MUST be .appimage since the official Ubuntu 26.04 still has OLD version

So for more future luxury when you still conditioned to type rpi-imager .. this below with start the new one everytime your conditioned finger or scripps call it:

`cd /usr/bin && sudo mv rpi-imager rpi-imager.old && sudo ln -s rpi-imager -> /opt/imager_2.0.11.1_amd64.AppImage`

1. **Select Device and OS:** Target Selection.
Launch Raspberry Pi Imager on your computer. Click **CHOOSE DEVICE** and select **Raspberry Pi Zero 2 W**. Click **CHOOSE OS** and select **Raspberry Pi OS** (64-bit or 32-bit Lite/Desktop).

To verify: Confirm that both "Raspberry Pi Zero 2 W" and your selected OS title are displayed in the main selection boxes.


2. **Select Target Storage:** Drive Selection.
Insert your microSD card into your computer's card reader. Click **CHOOSE STORAGE** and select your microSD card from the list.

To verify: Ensure the storage capacity and drive name displayed match your microSD card rather than an external backup drive.


3. **Open OS Customization:** Shortcut: Ctrl + Shift + X.
Press **Ctrl + Shift + X** on your keyboard (or click **NEXT** and select **EDIT SETTINGS** when the prompt appears) to open the OS Customization window.

To verify: Confirm that a modal window titled "OS Customization" opens displaying the **GENERAL** and **SERVICES** tabs.

4. **Configure Username and Wi-Fi Access:** GENERAL Tab.
In the **GENERAL** tab, set up your primary user and wireless configuration:

* **Set hostname**: Check this box and enter a hostname (e.g., `pi-zero2w.local`).
* **Set username and password**: Check this box and define your user account (the legacy default `pi`/`raspberry` login is disabled by default).
* **Configure wireless LAN**: Check this box and fill in your network credentials:
* **SSID**: Enter your Wi-Fi network name. *(Note: Raspberry Pi Zero 2 W only supports **2.4 GHz** Wi-Fi networks).*
* **Password**: Enter your Wi-Fi password.
* **Wireless LAN country**: Select your country code to enable the Wi-Fi radio under local regulations.

To verify: Double-check that your Wi-Fi network name and password are typed correctly and your country code is set.

5. **Configure SSH and Public Keys:** SERVICES Tab.
Select the **SERVICES** tab to set up remote command-line access:

* Check **Enable SSH**.
* Select **Allow public-key authentication only** for security, or choose **Use password authentication** to log in using the account password created in Step 4.
* If using public-key authentication, open a terminal on your host computer, copy the contents of your public SSH key file (typically `~/.ssh/id_rsa.pub` or `~/.ssh/id_ed25519.pub`), and paste the string into the **Authorized keys** field.

To verify: Ensure the "Enable SSH" checkbox is marked and your key string begins with `ssh-rsa` or `ssh-ed25519`.

6. **Save and Flash:** Writing to SD Card.
Click **SAVE** at the bottom of the OS Customization window, then click **WRITE** (or **NEXT** then **YES**). Confirm the warning prompt that all data on the SD card will be overwritten.

To verify: Wait until the write and verification progress bar reaches 100% and displays a "Write Successful" modal before removing the card.

Once finished, eject the microSD card, insert it into your Raspberry Pi Zero 2 W, and connect power. The device will automatically connect to your 2.4 GHz Wi-Fi network and apply your username and SSH configurations on first boot. Refer to the [Raspberry Pi Official Documentation](https://www.raspberrypi.com/documentation/computers/getting-started.html) for video guidance on software installation.

Auto Host Naming
The best approach is reading the Pi’s hardcoded CPU serial number at boot using a systemd service, then executing specific program logic based on a mapping configuration.

**1. Retrieve the Unique Hardware ID**
Every Raspberry Pi Zero 2 W has a unique SoC serial number available directly via Linux devicetree:

`cat /sys/firmware/devicetree/base/serial-number`

**2. Create the Router Script**
Write a Python script (e.g., `/usr/local/bin/board_router.py`) that reads the serial number on boot and launches the designated program or behavior:

```python
import os
import sys
import subprocess

def get_serial():
    try:
        with open('/sys/firmware/devicetree/base/serial-number', 'r') as f:
            return f.read().replace('\x00', '').strip()
    except Exception as e:
        sys.exit(f"Failed to read serial: {e}")

# Map Pi Serial Numbers to specific programs or commands
BOARD_CONFIGS = {
    "10000000a1b2c3d4": ["/usr/bin/python3", "/opt/apps/sensor_node.py"],
    "10000000e5f6a7b8": ["/usr/bin/python3", "/opt/apps/display_node.py"],
}

DEFAULT_CMD = ["/usr/bin/python3", "/opt/apps/default_node.py"]

serial = get_serial()
cmd = BOARD_CONFIGS.get(serial, DEFAULT_CMD)

# Replace current process with the target program
os.execv(cmd[0], cmd)

```

**3. Set Up a Systemd Boot Service**
Create `/etc/systemd/system/board-router.service`:

```ini
[Unit]
Description=Hardware-Specific Startup Launcher
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /usr/local/bin/board_router.py
Restart=on-failure
User=root

[Install]
WantedBy=multi-user.target

```

Enable the service once on the master card before cloning: `sudo systemctl enable board-router.service`.

**4. Master SD Card Preparation Checklist**
To prevent network and security conflicts across clones, perform these cleanup steps on the master card right before shut down and cloning:

* **Reset Systemd Machine ID**: Run `sudo truncate -s 0 /etc/machine-id` so each cloned Pi generates its own unique ID on boot.
* **Clear SSH Host Keys**: Delete `sudo rm /etc/ssh/ssh_host_*`. Add `dpkg-reconfigure openssh-server` to a first-boot script if you want new keys auto-generated upon clone startup.
* **Set Dynamic Hostname**: You can add `subprocess.run(["hostnamectl", "set-hostname", f"pi-{serial[:6]}"])` inside your router script to automatically name each Pi on boot.

Are you looking to launch completely different applications per board, or just pass unique configuration values (like IDs or local roles) to the same codebase?

Make it an AccessPoint

https://raspberrytips.com/access-point-setup-raspberry-pi/

#Headless ssid and pass change

Method 1: The Pre-configured NetworkManager File (Headless Fix)If you cannot plug the Pi into a monitor and keyboard, you can write a NetworkManager configuration file directly onto the SD card's boot partition. [1] (https://github.com/raspberrypi/trixie-feedback/issues/61)Safely remove the SD card from your Pi and plug it into your computer.Open the partition named bootfs (or boot).Create a new folder named conf inside the root of the boot partition if it doesn't already exist.Inside that conf folder, create a new file named wlan0 (with no file extension, or your operating system may require you to name it wlan0.nmconnection).Paste the following configuration, replacing YOUR_SSID and YOUR_PASSWORD with your actual network details:

 [connection]
 id=Preconfigured-WiFi
 type=wifi
 interface-name=wlan0

 [wifi]
 mode=infrastructure
 ssid=YOUR_SSID

 [wifi-security]
 auth-alg=open
 key-mgmt=wpa-psk
 psk=YOUR_PASSWORD

 [ipv4]
 method=auto

 [ipv6]
 method=auto


#Get OS to work with UART

On MORA, disable Bluetooth and dedicate the full UART to CATIA

Open and edit the boot configuration:

 sudo nano /boot/firmware/config.txt

Under [all] add:

   ```text
   enable_uart=1
   dtoverlay=disable-bt
   ```

Then run sudo raspi-config → Interface Options → Serial Port:

Serial login shell: No
Serial hardware: Yes

Reboot, then check:

## Preventing Sleep and Suspend States

Configure a headless Raspberry Pi Zero 2 W to run continuously without ever entering a sleep, suspend, hibernate, or standby state**.
It keeps the system awake indefinitely, when powerd, while allowing normal, safe hardware speeds.

This is need to avoid issues with e.g. a RPI AI Camera

**Step 1:** 
Block All Systemd Sleep & Suspend StatesThe most secure way to stop a Linux system from sleeping is to **mask** its power-saving targets. This completely disconnects the sleep commands from the operating system, making it impossible for background processes to trigger them.

Run the following command in your terminal:
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```
**Step 2:**
Disable Kernel-Level Console BlankingBy default, the Linux kernel triggers a low-power "blanking" standby mode after 10 minutes of inactivity. Even on a headless system, disabling this prevents the kernel from spinning down display architectures.
1. Open the boot command configuration file:
   ```bash
   sudo nano /boot/firmware/cmdline.txt
   ```
   *(Note: If you are using an older OS version, this file is located at `/boot/cmdline.txt`).*
2. Append the following argument to the end of the existing text block.
   **Crucial:** Keep everything on a **single, continuous line**. Do not press Enter or add a new line.
   ```text
   consoleblank=0
   ```

3. Save and exit (`Ctrl + O`, `Enter`, then `Ctrl + X`).

**Step 3:**
Ensure Normal, Non-Overclocked Settings (Optional)If you previously added aggressive hardware-forcing settings to your configuration file, you should remove them to let the Pi manage its temperatures naturally while staying awake.
1. Open the boot configuration file:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```

2. Look at the bottom of the file and ensure these lines are either **removed** or commented out with a `#`:
   ```ini
   # force_turbo=1
   # arm_freq_min=1000
   ```

3. Save and exit (`Ctrl + O`, `Enter`, then `Ctrl + X`).

**Step 4:** Apply ChangesReboot your Raspberry Pi to firmly lock all of these configurations into place:
```bash
sudo reboot
```

The Raspberry Pi Zero 2 W is now permanently awake and will never go to sleep!
