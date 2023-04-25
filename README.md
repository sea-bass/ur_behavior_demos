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

Set up pyrobosim and PDDLStream. Note we only pip install pyrobosim for its dependencies, but then rely on colcon build
to rebuild the actual module.

```
cd ~/ur_behavior_ws/src/ur_behavior_demos/dependencies/pyrobosim && \
    pip3 install ./pyrobosim && \
    setup/setup_pddlstream.bash && \
    pip3 uninstall -y pyrobosim && \
    source ./setup/source_pyrobosim.bash
```

Build and source the workspace:

```
cd ~/ur_behavior_ws/
colcon build
source install/setup.bash
```

### Docker Setup (TODO)

---

## Running Examples

First, check that basic setup worked by launching the UR MoveIt demo launch file:

```
ros2 launch ur_behavior ur_behavior_demo.launch.py ur_type:=ur5e use_fake_hardware:=true
```

Load the pyrobosim world and its ROS server interface:

```
ros2 run ur_behavior load_pyrobosim.py
```

Load a pyrobosim planning server and make a pyrobosim planning request:

```
cd ~/ur_behavior_ws/src/ur_behavior_demos/dependencies/pyrobosim && \
    source ./setup/source_pyrobosim.bash
ros2 run ur_behavior demo_planning.py
```
