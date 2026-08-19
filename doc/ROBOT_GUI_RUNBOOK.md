# Robot GUI integration runbook

## Fixed network settings

- Ubuntu ROS PC: `172.21.50.85/16`
- Windows GUI PC: `172.21.60.68`
- `tending_bridge`: TCP `5901`, only `172.21.60.68`, one remote client

## Pull and build on Ubuntu

```bash
cd /home/mkketi/dev_ws/NCC/Tending_ws/src/KETI_Tending_Robot
git status
git pull --ff-only origin main

cd /home/mkketi/dev_ws/NCC/Tending_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  tending_interfaces tending_camera tending_data tending_control tending_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select tending_control tending_bridge
colcon test-result --verbose
```

```bash
sudo ufw allow from 172.21.60.68 to any port 5901 proto tcp
```

## Start on Ubuntu

Use three terminals and export `ROS_LOCALHOST_ONLY=1` in all of them.

```bash
# Terminal A
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/mkketi/dev_ws/NCC/Tending_ws/install/setup.bash
ros2 launch tm_driver tm_bringup.launch.py <TM5_IP>
```

```bash
# Terminal B: camera is deferred, so keep the stub enabled
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/mkketi/dev_ws/NCC/Tending_ws/install/setup.bash
ros2 launch tending_control mvp_inspect.launch.py \
  use_stub:=true ignore_safety_flags:=false
```

```bash
# Terminal C
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /home/mkketi/dev_ws/NCC/Tending_ws/install/setup.bash
ros2 launch tending_bridge bridge.launch.py
```

## Windows commands

```powershell
cd C:\Users\kming\dev_ws\NCC\KETI_CNCTendingSystem
git status
git pull --ff-only origin main
msbuild .\KETI_CNCTendingRobot\KETI_CNCTendingRobot.csproj `
  /t:Rebuild /p:Configuration=Release /p:Platform=x64

ping 172.21.50.85
Test-NetConnection 172.21.50.85 -Port 5901
```

GUI bridge settings: `ROS_BRIDGE`, `172.21.50.85`, TCP `5901`, ping `1000 ms`,
reconnect `2000 ms`.

Before full inspection, teach and collision-check `approach`, `inspect`, and `retreat`.
Start with STOP verification, taught `home`, then one J6 jog (`3 deg`, `5 deg/s`). Never
use `ignore_safety_flags:=true` with the physical robot, and never enable
`DIRECT_TMSVR` while `tm_bringup` owns the robot connection.

