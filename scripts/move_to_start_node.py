#!/usr/bin/env python

import sys
import rospy as ros
from actionlib import SimpleActionClient
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from controller_manager_msgs.srv import SwitchController
from franka_msgs.msg import ErrorRecoveryActionGoal
from franka_gripper.msg import MoveAction, MoveGoal
import argparse


class MoveToStartNode():
    def __init__(self, args):
        ros.init_node('move_to_start_node')
        self.switch_controller = args.switch_controller
        self.client = SimpleActionClient("/effort_joint_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.client.wait_for_server()
        self.gripper_client = SimpleActionClient("/franka_gripper/move", MoveAction)
        self.gripper_client.wait_for_server()
        self.joint_pose = ros.get_param("joint_pose", None)
        
        self.spacemouse_sub = ros.Subscriber(
            "/spacenav/joy",
            Joy,
            self.joy_callback,
            queue_size=1,
        )
        self.spacemouse_sub = ros.Subscriber(
            "/policy/reset",
            Empty,
            self.policy_callback,
            queue_size=1,
        )
        self.error_recovery_pub = ros.Publisher("/franka_control/error_recovery/goal", 
                                                ErrorRecoveryActionGoal, queue_size=1)
        ros.spin()
    
    def joy_callback(self, msg):
        if msg.buttons[13] > 0:
            # R button on space mouse
            # do not move to start if the robot is close to joint limits
            # use programming mode to manually steer the robot close to default position
            # then move to start
            self.move_to_start()
        if msg.buttons[12] > 0:
            self.error_recovery_pub.publish(ErrorRecoveryActionGoal())
    
    def policy_callback(self, msg):
        self.move_to_start()
            
    def switch_controllers(self, start_controllers, stop_controllers, strictness=2):
        ros.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_service = ros.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            response = switch_service(
                start_controllers=start_controllers,
                stop_controllers=stop_controllers,
                strictness=strictness,  # 1 = BEST_EFFORT, 2 = STRICT
                start_asap=True,
                timeout=5.0
            )
            if response.ok:
                ros.loginfo("Switched controllers successfully.")
            else:
                ros.logwarn("Failed to switch controllers.")
        except ros.ServiceException as e:
            ros.logerr(f"Service call failed: {e}")
    
    def move_to_start(self):
        if len(self.switch_controller) > 0:
            self.switch_controllers(
                ["effort_joint_trajectory_controller"],
                [self.switch_controller]
            )
        
        move_goal = MoveGoal()
        move_goal.width = 0.08
        move_goal.speed = 0.1
        self.gripper_client.send_goal(move_goal)
        
        joint_state = ros.wait_for_message("/franka_state_controller/joint_states", JointState)
        initial_pose = dict(zip(joint_state.name, joint_state.position))

        max_movement = max(abs(self.joint_pose[joint] - initial_pose[joint]) for joint in self.joint_pose)
        point = JointTrajectoryPoint()
        point.time_from_start = ros.Duration.from_sec(
            # Use either the time to move the furthest joint with 'max_dq' or 500ms,
            # whatever is greater
            max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
        )
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*self.joint_pose.items())]
        point.velocities = [0] * len(self.joint_pose)

        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

        ros.loginfo('Sending trajectory Goal to move into initial config')
        self.client.send_goal_and_wait(goal)

        result = self.client.get_result()
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            ros.logerr('move_to_start: Movement was not successful: ' + {
                FollowJointTrajectoryResult.INVALID_GOAL:
                """
                The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
                Is the 'joint_pose' reachable?
                """,

                FollowJointTrajectoryResult.INVALID_JOINTS:
                """
                The joint pose you specified is for different joints than the joint trajectory controller
                is claiming. Does you 'joint_pose' include all 7 joints of the robot?
                """,

                FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                """
                During the motion the robot deviated from the planned path too much. Is something blocking
                the robot?
                """,

                FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                """
                After the motion the robot deviated from the desired goal pose too much. Probably the robot
                didn't reach the joint_pose properly
                """,
            }[result.error_code])

        else:
            ros.loginfo('move_to_start: Successfully moved into start pose')
    
        if len(self.switch_controller) > 0:
            self.switch_controllers(
                [self.switch_controller],
                ["effort_joint_trajectory_controller"]
            )
            

if __name__=="__main__":
    parser = argparse.ArgumentParser(description="Move to start node")
    parser.add_argument("--switch_controller", default="", type=str, help="controller to switch")
    args, _ = parser.parse_known_args()
    
    MoveToStartNode(args)
