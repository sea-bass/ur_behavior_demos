#!/usr/bin/env python3

"""
Script that performs full Task and Motion Planning (TAMP) with PDDLStream.
"""

import os
import rclpy

from pyrobosim.utils.pose import Pose
from pyrobosim_msgs.msg import TaskPlan
from pyrobosim_msgs.srv import RequestWorldState
from pyrobosim_ros.ros_conversions import task_plan_to_ros
from pyrobosim_ros.ros_interface import update_world_from_state_msg
from pyrobosim.core import Robot
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.planning import PDDLStreamPlanner
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from utils import (
    create_world_from_yaml,
    get_domains_folder,
    get_pddlstream_mapping_functions,
)


class PlannerNode(Node):
    def __init__(self):
        super().__init__("demo_planner")
        self.declare_parameter("world_file", value="test_world")

        # Load world
        world_file = self.get_parameter("world_file").value
        self.world = create_world_from_yaml(world_file)

        # Add robot with grasp generator
        grasp_props = ParallelGraspProperties(
            max_width=0.175,
            depth=0.1,
            height=0.04,
            width_clearance=0.01,
            depth_clearance=0.01,
        )
        self.robot = Robot(
            name="ur_robot",
            grasp_generator=GraspGenerator(grasp_props)
        )
        self.world.add_robot(self.robot, loc="kitchen", pose=Pose(x=1.0, y=-3.0))

        # Create planner
        stream_info_fn, stream_map_fn = get_pddlstream_mapping_functions()
        self.planner = PDDLStreamPlanner(
            self.world,
            get_domains_folder(),
            stream_info_fn=stream_info_fn,
            stream_map_fn=stream_map_fn,
        )
        self.planning = False

        # Publisher for a task plan
        self.plan_pub = self.create_publisher(
            TaskPlan, "commanded_plan", 10)
        
        # Service client for world state
        self.world_state_client = self.create_client(
            RequestWorldState, "request_world_state"
        )
        self.world_state_future_response = None
        while not self.world_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for world state server...")

        # Main loop
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def request_world_state(self):
        """Requests the latest state from the world."""
        self.planning = True
        self.get_logger().info("Requesting world state...")
        self.world_state_future_response = self.world_state_client.call_async(
            RequestWorldState.Request()
        )


    def plan(self, goal):
        """ Plans using PDDLStream"""
        self.get_logger().info("Planning...")
        
        # Plan for a solution
        plan = self.planner.plan(
            self.robot,
            goal,
            search_sample_ratio=0.5,
            planner="ff-astar",
            max_planner_time=15.0,
            max_attempts=2,
        )
        self.get_logger().info(f"{plan}")

        # Validate and maybe publish the plan
        if plan is None:
            self.get_logger().info("Planning failed")
            return False
        self.get_logger().info("Publishing plan...")
        plan_msg = task_plan_to_ros(plan)
        self.plan_pub.publish(plan_msg)
        return True

    def timer_callback(self):
        if not self.planning:
            self.request_world_state()

        ready_to_plan = (
            self.world_state_future_response
            and self.world_state_future_response.done()
        )
        if ready_to_plan:
            result = self.world_state_future_response.result()
            update_world_from_state_msg(self.world, result.state)
            self.plan(goal)


if __name__ == "__main__":
    rclpy.init()
    planner_node = PlannerNode()

    executor = MultiThreadedExecutor(num_threads=os.cpu_count())
    executor.add_node(planner_node)

    # Create a dummy goal.
    # TODO: This will later come from another node that accepts a goal.
    goal = [
        ("Holding", "ur_robot", "banana0"),
        ("At", "ur_robot", "kitchen"),
    ]

    try:
        executor.spin()
    finally:
        executor.shutdown()
        planner_node.destroy_node()
        rclpy.shutdown()
