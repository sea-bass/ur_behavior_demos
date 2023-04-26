# ur_behavior_demos
Behavior examples for Universal Robots (UR) manipulators.

## Installation

### Local Setup

This repo is being developed with Ubuntu 22.04 on ROS 2 Rolling.

Make a colcon workspace, e.g.:

```
mkdir -p ~/ur_behavior_ws/src
cd ~/ur_behavior_ws/src
```

Clone this repo with submodules:

```
git clone --recurse-submodules https://github.com/sea-bass/ur_behavior_demos.git
```

Install dependencies:

```
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build and source the workspace:

```
colcon build --symlink-install
source install/setup.bash
```

### Docker Setup (TODO)

---

## Running Examples

First, check that basic setup worked by launching the UR MoveIt demo launch file:

```
ros2 launch ur_behavior ur_behavior_demo.launch.py
```
