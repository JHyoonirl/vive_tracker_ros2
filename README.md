# Vive Tracker ROS 2

Reusable Vive Tracker pose driver for Linux and ROS 2 Jazzy. It uses
`libsurvive` and its Python bindings, so SteamVR and VRPN are not required.

## Features

- Python API for reading one or more Vive Tracker poses.
- ROS 2 node publishing `geometry_msgs/msg/PoseStamped`.
- Separate raw and transformed pose topics.
- Tracker serial discovery utility.
- Configurable frame, publish rate, and rigid-body transform.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- A Vive Tracker, a compatible USB receiver, and at least one Lighthouse
- `build-essential`, `cmake`, `libusb-1.0-0-dev`, `libudev-dev`, and Python 3

## Get the source

```bash
git clone --recursive https://github.com/JHyoonirl/vive_tracker_ros2.git
cd vive_tracker_ros2
```

If the repository was cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

## Build

```bash
sudo apt-get install build-essential cmake libusb-1.0-0-dev libudev-dev python3-dev python3-pip
./scripts/build_libsurvive.sh
python3 -m pip install -e . --user

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The build script keeps `libsurvive` build output inside the submodule. The
generated binaries are ignored by Git.

## Find a Tracker serial number

Turn on the Tracker and place it where a Lighthouse can see it:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
vive-tracker-serial
```

The output contains an `LHR-...` serial. Use that value in the node command or
in a YAML parameter file. Do not commit real device serials to this repository.

## Run the ROS 2 node

For one Tracker:

```bash
ros2 run vive_tracker_ros vive_tracker_node --ros-args \
  -p serial_numbers:="['LHR-XXXXXXXX']" \
  -p tracker_ids:="['tracker']" \
  -p namespaces:="['vive']" \
  -p publish_rate:=100.0 \
  -p frame_id:=world
```

For two Trackers:

```bash
ros2 run vive_tracker_ros vive_tracker_node --ros-args \
  -p serial_numbers:="['LHR-LEFT','LHR-RIGHT']" \
  -p tracker_ids:="['left','right']" \
  -p namespaces:="['vive','vive']"
```

Each Tracker publishes:

- `/<namespace>/<tracker_id>/pose_raw`: pose reported by `libsurvive`
- `/<namespace>/<tracker_id>/pose`: pose after the configured transform

Both topics use `geometry_msgs/msg/PoseStamped`. The default transform is an
identity transform. Configure translations in metres and roll, pitch, yaw in
degrees with `config/tracker.example.yaml`.

## Python API

```python
from vive_tracker import ViveTracker

tracker = ViveTracker(serial_numbers=["LHR-XXXXXXXX"])
tracker.connect()
try:
    pose = tracker.get_pose("LHR-XXXXXXXX")
    if pose is not None:
        print(pose.position, pose.orientation)
finally:
    tracker.disconnect()
```

The Python API has no dependency on `rclpy` or `geometry_msgs`.

## Coordinate frames

`libsurvive` estimates a `world` frame from the Lighthouse scene. It is not
automatically aligned with a robot base frame. The `pose_raw` topic exposes the
Tracker pose without an application-specific offset. The `pose` topic applies
the configured rigid-body transform to that pose.

## License

The driver code is MIT licensed. `libsurvive` remains a separate MIT-licensed
upstream submodule; see `third_party/libsurvive/LICENSE` after initialization.
