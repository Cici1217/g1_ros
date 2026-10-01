# Official Gear Sonic Body + ROS Optical Hands

This is the recommended PICO real-robot configuration for this package.

- Body: the official `pico_manager_thread_server.py` converts PICO body posture
  and publishes the official `pose` protocol directly to Gear Sonic over ZMQ.
- Hands: `g1_pico_teleop` reads only the two OpenXR 26-joint optical hands and
  publishes ROS `JointTrajectory` commands to the existing Inspire controllers.
- The hand process never publishes body targets, never switches a Gear Sonic
  mode, and never calls a Gear Sonic service.

The old `pico_teleop.launch.py` / `pico_ros_adapter` combined entry has been
removed. The only ROS adapter entry is `pico_hand_adapter`, launched through
`pico_hands.launch.py`. Historical body-mapping references remain only in
`test/legacy/` for regression tests; they are not installed or used at runtime.

```text
g1_pico_teleop/
├── launch/pico_hands.launch.py       # Hand-only launch
├── scripts/pico_xrt_source.py        # Optical-hand source (--hands-only)
├── g1_pico_teleop/
│   ├── hand_ros_adapter_node.py      # UDP -> hand JointTrajectory
│   ├── optical.py                    # OpenXR 26 joints -> Inspire native6
│   ├── hand_command.py               # ROS joint order, limits and slew rate
│   └── wire.py                       # Compatible UDP v1 encoding/decoding
├── config/pico_native6_calibration.json
└── test/                            # Current tests + historical references
```

The source's compatible body-capture option and UDP v1 body-sized field remain
unchanged; use `--hands-only` as below. Neither is a ROS body-control path.

## 1. Architecture and the ZMQ question

```text
PICO headset
    |
XRoboToolkit PC Service
    |-- SDK client 1: official PICO manager
    |       `-- body/SMPL conversion from the official implementation
    |       `-- ZMQ PUB :5556 --> Gear Sonic SUB --> G1 body
    |
    `-- SDK client 2: pico_xrt_source.py --hands-only
            `-- local UDP :5570 --> pico_hand_adapter
                    `-- ROS JointTrajectory --> Inspire hand controllers
```

ZMQ PUB/SUB supports multiple subscribers, but the official `pose` message does
not contain the raw OpenXR 26-joint optical hands. Its legacy
`left_hand_joints` and `right_hand_joints` fields are seven-dimensional values
derived from controller trigger/grip input. They cannot replace the optical
hand data used by this package. Therefore, a second subscriber to the official
`pose` topic alone cannot drive the current optical-hand retargeter.

Because this change is restricted to `g1_ros`, the hand source is a second
XRoboToolkit SDK client instead. The installed PICO client library connects to
the PC Service; it is not itself a ZMQ publisher. Concurrent SDK-client behavior
must be confirmed once with the live headset before treating this as a passed
hardware configuration.

## 2. Build once

```bash
cd /home/cici/projects/GrootReproucing/g1_full_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select g1_pico_teleop
```

When upgrading an existing workspace, an incremental build may leave the old
installed executable or launch file behind. Use a clean package build/install
or a fresh workspace before deployment, and check
`ros2 pkg executables g1_pico_teleop`: `pico_ros_adapter` must no longer be listed. Do not run stale
copies of the removed combined entry.

Every workstation ROS terminal uses:

```bash
source /opt/ros/jazzy/setup.bash
source /home/cici/projects/GrootReproucing/g1_full_ws/install/setup.bash
export ROS_DOMAIN_ID=164
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
```

## 3. Terminal 1: XRoboToolkit PC Service

```bash
cd /opt/apps/roboticsservice

env \
  LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib:/opt/apps/roboticsservice/SDK/x64" \
  QT_PLUGIN_PATH="/opt/apps/roboticsservice/plugins/${QT_PLUGIN_PATH:+:$QT_PLUGIN_PATH}" \
  QT_QML_PATH="/opt/apps/roboticsservice/qml/${QT_QML_PATH:+:$QT_QML_PATH}" \
  ./RoboticsServiceProcess
```

Connect the PICO Robotics Streaming application and wait for `WORKING`. Confirm
that body tracking, both foot trackers, and both optical hands are available.

Find the workstation address reachable from the robot:

```bash
ROBOT_IP=10.42.0.1
WORKSTATION_IP=$(ip route get "$ROBOT_IP" | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}')
echo "$WORKSTATION_IP"
```

## 4. Terminal 2: robot ROS hand hardware

The prebuilt ROS container provides the existing Inspire hardware interface and
controllers. It does not own the Gear Sonic body path in this configuration.

```bash
ssh -t unitree@10.42.0.1

sudo systemctl stop ros2-g1-gear-sonic-bringup.service gear-sonic.service
sudo systemctl start inspire-g1.service

docker rm -f g1-bringup 2>/dev/null || true
/home/unitree/.local/bin/rocker \
  --mode non-interactive --name g1-bringup \
  --env ROS_DOMAIN_ID=164 \
  --env FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  --network host --ipc host --privileged \
  ghcr.io/mqcmd196/g1_ros:rosbag \
  "bash -lc 'source /root/target_ws/install/setup.bash && exec ros2 launch g1_bringup g1_bringup.launch.py hand_type:=inspire_dfq network_interface:=enP8p1s0 use_gear_sonic:=false use_rviz:=false use_d435i:=false'"
```

Keep this terminal open. Do not publish `/cmd_vel` or call locomotion services
from this ROS instance while the official Gear Sonic process owns the body.

## 5. Terminal 3: official Gear Sonic deploy on the robot

Replace `<WORKSTATION_IP>` with the value printed in terminal 1.

```bash
ssh -t unitree@10.42.0.1

sudo systemctl stop gear-sonic.service
cd /home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh
./deploy.sh --input-type zmq --zmq-host <WORKSTATION_IP> real
```

This is the official direct PICO body consumer. It connects to the workstation
ZMQ publisher; it does not connect to `g1_ros` for body posture.

## 6. Terminal 4: official PICO body manager on the workstation

Use the official PICO manager entry point and its original body conversion.
The removed ROS combined entry is not part of this body path.

```bash
cd /home/cici/projects/GrootReproucing/GR00T-WholeBodyControl
source .venv_teleop/bin/activate

python -B gear_sonic/scripts/pico_manager_thread_server.py \
  --manager \
  --port 5556 \
  --num_frames_to_send 5 \
  --target_fps 50 \
  --cuda
```

Use the official body calibration and mode controls: stand in the official
calibration pose, press PICO `A+B+X+Y`, then press `A+X` to enter POSE. Confirm
that body frames continue advancing. In terminal 3, press `]` once to enter
policy control, then press Enter once to enable ZMQ streaming.

> The upstream manager uses the PICO controllers for calibration and mode
> switching. If the experiment is strictly controller-free, the unmodified
> upstream workflow cannot be started as documented. The existing
> `pico_native6_manager --body-only` terminal gate in the other workspace keeps
> the official body conversion but is not byte-for-byte upstream. Do not hide
> this distinction when reporting which configuration was tested.

## 7. Terminal 5: hand-only ROS adapter on the workstation

```bash
source /opt/ros/jazzy/setup.bash
source /home/cici/projects/GrootReproucing/g1_full_ws/install/setup.bash
export ROS_DOMAIN_ID=164
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA

ros2 launch g1_pico_teleop pico_hands.launch.py start_xrt_source:=false
```

The log must say:

```text
Hand-only adapter ... output is disabled
```

It must not mention `SmplMotion`, `enable_smpl_stream`, body mapping, or Gear
Sonic control.

## 8. Terminal 6: PICO optical-hand source on the workstation

```bash
env PYTHONDONTWRITEBYTECODE=1 \
  /home/cici/projects/GrootReproucing/GR00T-WholeBodyControl/.venv_teleop/bin/python -B \
  /home/cici/projects/GrootReproucing/g1_full_ws/install/g1_pico_teleop/lib/g1_pico_teleop/pico_xrt_source.py \
  --hands-only
```

The first status line must contain `mode=hands-only`; subsequent lines must show
a nonzero `rate` and `left=ok right=ok`. In this mode the source does not call
`get_body_joints_pose()`.

## 9. Check and enable only the hands

In another workstation ROS terminal:

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
```

Expected controller states:

```text
left_hand_controller   active
right_hand_controller  active
upper_body_controller  inactive
joint_state_broadcaster active
```

Keep both real hands visible and enable only the hand adapter:

```bash
ros2 service call /pico_hand_adapter/enable \
  std_srvs/srv/SetBool '{data: true}'
```

The adapter seeds its slew limiter from measured `/joint_states`, preserves the
existing native6 mapping and joint order, and holds both targets if either
optical hand disappears.

Disable only the hands with:

```bash
ros2 service call /pico_hand_adapter/enable \
  std_srvs/srv/SetBool '{data: false}'
```

This service never changes the Gear Sonic body state.

## 10. Acceptance checks

Before free motion, keep the robot on the gantry and verify:

1. With terminals 1-4 running and terminals 5-6 stopped, official body tracking
   behaves exactly as before.
2. Starting terminals 5-6 while hand output is disabled does not change the
   body posture.
3. Enabling `/pico_hand_adapter/enable` changes only the 12 Inspire joints.
4. Removing either hand from view holds both hands and body tracking continues.
5. Stopping the hand source disables hand output after the source timeout and
   does not send a Gear Sonic stop command.
6. Both SDK clients continue receiving new timestamps for at least two minutes.

If item 6 fails, the installed XRoboToolkit service does not support the two
concurrent SDK clients reliably. A true one-PUB/two-SUB design would then
require the official PICO publisher to expose raw OpenXR hand poses; that change
cannot be implemented solely inside `g1_ros`.

## 11. Shutdown

1. Disable `/pico_hand_adapter/enable`.
2. Stop terminal 6, then terminal 5.
3. Stop official body streaming using its normal terminal-3 procedure.
4. Stop terminal 4, then terminal 3.
5. Stop the ROS container and Inspire service:

```bash
ssh unitree@10.42.0.1 'docker stop -s SIGINT --time 30 g1-bringup'
ssh -t unitree@10.42.0.1 'sudo systemctl stop inspire-g1.service'
```

Do not run stale copies of the removed combined launch, another hand demo, or another publisher
to either hand-controller trajectory topic at the same time.
