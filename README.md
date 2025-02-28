# Path Planning for Drones

## How to clone this repo correctly

To clone this repo correctly, you should firstly clone it like this:

```bash
git clone https://github.com/thomasvandenheede/paparazzi-Group6
```

after this please change directory into the repo and checkout to this branch:

```bash
cd /paparazzi-Group6
git checkout 1_path_planning
```

Perfect! Now don't forget to download the sub-modules like this:

```bash
git submodule update --init --recursive
```

This will ensure that all the files within the /ext (external) are correctly downloaded. The last thing to do is make the project by:

```bash
make
```

after this you should be ready to run paparazzi.

## Start running paparazzi

To run this repo, start by starting paparazzi:

```bash
sudo ./start.py
```

and select course_conf.xml and course_control_panel.xml.

## Info about GCS (telemetry data)

After getting into GCS (paparazzi telemetry GUI), some interesting things:

- Within the environment, use F to center the camera.
- To record, in GCS on the top left there is a "start recording button".
- To get Telemetry data logged, go to the "Settings" tab at the bottom and press telemetry record START.
- If the drone died (killed or otherwise), your Throttle should be 0% and red, to startup again. Press the green resurrect button next to the red kill button.

### Where to find the logs

The logs you just recorded can be found within your Files (or Documents), then on the left you should see the IP adress of the drone (if you are connected to it over WiFi). Press it and there you will find a telemtry and images folder. Just copy and paste those to your own directory.

## Camera info

To get a live feed from the camera (different then logging the camera), you can toggle this within Gazebo or via the command line, using:

```bash
ffplay -vf "transpose=2, scale=2*iw:-1" -i ./sw/tools/rtp_viewer/rtp_5000.sdp -protocol_whitelist "file,crypto,data,rtp,udp" -fflags nobuffer -flags low_delay
```

---
