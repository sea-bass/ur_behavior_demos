#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import rclpy
import sys
import threading

from tiago_interface import TIAGoRobot
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper
from utils import create_world_from_yaml


def create_ros_node():
    """ Initializes ROS node """
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1)
    node.declare_parameter("world_file", value="test_world")
    
    # Set the world
    world_file = node.get_parameter("world_file").value
    if world_file == "":
        node.get_logger().error("A world_file parameter was not given.")
        sys.exit()
    else:
        node.get_logger().info(f"Using world file {world_file}")
        w = create_world_from_yaml(world_file)
    
    # Set the robot
    # TODO: Make sure the pose matches the initial pose set
    tiago = TIAGoRobot()
    w.add_robot(tiago, loc="kitchen", pose=Pose(x=1.0, y=-3.0))

    # ROS setup
    node.set_world(w)
    tiago.create_ros_interfaces()
    tiago.tuck_arm()

    return node


if __name__ == "__main__":
    n = create_ros_node()
    n.start()
