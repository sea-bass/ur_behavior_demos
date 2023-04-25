# UR ROS Interface

import rclpy

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from transforms3d.euler import euler2quat
from transforms3d._gohlketransforms import decompose_matrix, inverse_matrix
from transforms3d.quaternions import quat2mat

""" MoveIt """
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive

""" pyrobosim """
from pyrobosim.core import Robot
from pyrobosim.utils.pose import Pose


class UrRobot(Robot):
    """ A specific robot interface for UR. """
    def __init__(self): 
        super().__init__(self)
        self.name = "ur"
        self.radius = 0.2
        self.base_link_name = "base_link"


    def create_ros_interfaces(self):
        """ Creates ROS interfaces (call after adding robot to world) """
        self.node = self.world.ros_node
        # For sending commands directly to ros2_control, no MoveIt
        self.arm_ros_control_client = ActionClient(
            self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        # For MoveIt motion planning
        self.move_group_client = ActionClient(
            self.node, MoveGroup, "/move_action"
        )

        # Service client to receive a MoveIt planning scene message
        self.get_planning_scene_node = rclpy.create_node("get_planning_scene_node")
        self.get_planning_scene_client = self.get_planning_scene_node.create_client(GetPlanningScene, "/get_planning_scene")


    def execute_action(self, action, blocking=False):
        """ Wrapper to execute actions through the ROS interface """
        if action.type == "pick":
            self.pick_complete = False

            move_group_goal = self.fill_moveit_pose_planning_request(action.pose)
            self.pick_goal_future = self.move_group_client.send_goal_async(move_group_goal)
            self.pick_goal_future.add_done_callback(self.pick_goal_callback)

            rate = self.node.create_rate(2.0)
            while not self.pick_complete:
                rate.sleep()
            success = self.pick_status == GoalStatus.STATUS_SUCCEEDED
            self.node.get_logger().info(f"Pick completed!")
            return success

        # TODO: Implement place action overrides.


    def pick_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Planning goal rejected")
        else:
            self.node.get_logger().info("Planning goal accepted")

        self.pick_result_future = goal_handle.get_result_async()
        self.pick_result_future.add_done_callback(self.pick_result_callback)


    def pick_result_callback(self, future):
        self.pick_complete = True
        self.pick_status = future.result().status


    def fill_moveit_pose_planning_request(self, pose):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_link_name
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_pose.pose.position.x = pose.x
        target_pose.pose.position.y = pose.y
        target_pose.pose.position.z = pose.z
        target_pose.pose.orientation.w = pose.q[0]
        target_pose.pose.orientation.x = pose.q[1]
        target_pose.pose.orientation.y = pose.q[2]
        target_pose.pose.orientation.z = pose.q[3]

        move_group_goal = MoveGroup.Goal()

        move_group_goal.request.group_name = "arm"
        move_group_goal.request.num_planning_attempts = 10
        move_group_goal.request.max_velocity_scaling_factor = 1.0
        move_group_goal.request.max_acceleration_scaling_factor = 1.0
        move_group_goal.request.allowed_planning_time = 1.0
        move_group_goal.request.pipeline_id = "ompl"
        move_group_goal.request.planner_id = "RRTConnectkConfigDefault"

        # Use the current robot state as the start state
        move_group_goal.request.start_state.is_diff = False

        # Position target
        # https://github.com/ros-planning/moveit2/blob/bbb9264b72300a1c20ebe35c0b482685bd674752/moveit_core/kinematic_constraints/src/utils.cpp
        constraint_msg = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.link_name = "arm_tool_link"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        solid_primitive_msg = SolidPrimitive()
        solid_primitive_msg.type = SolidPrimitive.SPHERE
        # https://github.com/ros2/common_interfaces/blob/master/shape_msgs/msg/SolidPrimitive.msg
        solid_primitive_msg.dimensions = [0.02] # Tolerance: radius of a sphere in meters
        position_constraint.constraint_region.primitives = [solid_primitive_msg]
        position_constraint.header.frame_id = self.base_link_name
        position_constraint.header.stamp = self.node.get_clock().now().to_msg()
        position_constraint.constraint_region.primitive_poses = [target_pose.pose]
        position_constraint.weight = 1.0
        constraint_msg.position_constraints = [position_constraint]

        # Orientation target
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "arm_tool_link"
        orientation_constraint.header = position_constraint.header
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        constraint_msg.orientation_constraints = [orientation_constraint]
        move_group_goal.request.goal_constraints = [constraint_msg]

        move_group_goal.planning_options.planning_scene_diff.is_diff = False
        move_group_goal.planning_options.planning_scene_diff.robot_state.is_diff = False

        return move_group_goal
