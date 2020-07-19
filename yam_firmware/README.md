#yam_firmware

## Navigation Firmware

## Frontpanel Firmware

## Raspberry Pi Firmware
Low-level raspberry pi firmware `raspberry_pi_firmware.py` necessary to turn on/off the pi from the
front-panel (without ROS) and low-level OLED display for IP address.

### Installation

#### 1. Setting permission
Need to set permission of sudo to no-password for shutdown.

This can be done by modifying `/etc/sudoers`.

Enter the following into the terminal,

```
sudo nano /etc/sudoers
```

Then enter the following line at the end of the file,

```
<USER> ALL=NOPASSWD: /sbin/halt, /sbin/reboot, /sbin/poweroff, /sbin/shutdown
```

Where `<USER>` is your username on the raspberry pi.

#### 2. Running on startup
Add the script to cron. Start by editing the job entry

```
crontab -e
```

Add the following,

```
@reboot sleep 15;<PATH_TO_CATKIN_WS>/src/YAM/yam_firmware/src/raspberry_pi_firmware.py
```

Where `<PATH_TO_CATKIN_WS>` is the path to your ROS catkin workspace.

### How it works

## IMU Firmware
