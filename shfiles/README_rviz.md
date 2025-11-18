# Remote RViz (display on PC, ROS master on Pi)

This short guide explains how to run RViz on a remote PC while keeping the ROS master and computation (VINS-Fusion, flight planner) on the onboard Raspberry Pi.

Quick summary

- Start `roscore` and your compute nodes (VINS-Fusion, planner) on the Pi.
- On the PC, set `ROS_MASTER_URI` to the Pi and set `ROS_IP` to the PC's own IP.
- Start `rviz` on the PC; it will subscribe to topics from the Pi.

Example (replace IP addresses):

On the Pi (master):
```bash
export ROS_MASTER_URI="http://128.189.246.106:11311"
export ROS_IP="128.189.246.106"
roscore &
# start VINS-Fusion and planner as usual
roslaunch vins vins.launch &
roslaunch planner planner.launch &
```

On the PC (viewer):
```bash
export ROS_MASTER_URI="http://128.189.246.106:11311"
export ROS_IP="206.87.209.35"
rostopic list    # should show topics from the Pi
rviz -d /path/to/config.rviz
```

Using the included helper script

- `rviz_remote.sh <ROS_MASTER_IP> [LOCAL_IP] [RVIZ_CONFIG]`
- Example: `./rviz_remote.sh 128.189.246.106 206.87.209.35 ~/my_config.rviz`

Notes and troubleshooting

- Prefer `ROS_IP` over `ROS_HOSTNAME` unless you have working DNS/hosts mapping.
- If you don't see topics on the PC, check `env | grep ROS` on both machines.
- Firewalls can block ROS traffic. For quick testing, disable the firewall or allow traffic between the two hosts.
- If using Windows, prefer an Ubuntu PC or WSL2 with X server for RViz; network behavior may differ in WSL2.
To avoid editing files, use the `server_pi.sh` (run on the Pi) and `client_pc.sh` (run on the PC) scripts in this folder. They set `ROS_IP` and `ROS_MASTER_URI` and write them to `~/.bashrc`.
To avoid editing files, use the `server_pi.sh` (run on the Pi) and `client_pc.sh` (run on the PC) scripts in this folder. They set `ROS_IP` and `ROS_MASTER_URI` and write them to `~/.bashrc`.

Scripts and hardcoded IPs (edit these files before running):

- **`server_pi.sh` (Pi / ROS master):** `ONBOARD_IP=128.189.246.106` — sets `ROS_MASTER_URI=http://128.189.246.106:11311` and `ROS_IP=128.189.246.106`.
- **`client_pc.sh` (PC / RViz):** `ONBOARD_IP=128.189.246.106`, `GROUND_IP=206.87.209.35`, `ONBOARD_ALIAS=icondrone` — sets `ROS_MASTER_URI=http://128.189.246.106:11311`, `ROS_IP=206.87.209.35`, and adds `/etc/hosts` mapping `128.189.246.106 icondrone`.
- **`rviz_remote.sh` (one-shot):** runs RViz for the session and sets `ROS_MASTER_URI`/`ROS_IP` without modifying files.

Advanced: hostname mapping

- If nodes advertise hostnames (not IPs), add the Pi hostname mapping to `/etc/hosts` on the PC: `sudo sh -c 'echo "128.189.246.106 icondrone" >> /etc/hosts'`.
