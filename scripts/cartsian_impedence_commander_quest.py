#!/usr/bin/env python3
import rospy
import numpy as np
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from quest2ros.msg import OVR2ROSInputs
from franka_gripper.msg import GraspAction, GraspGoal
from scipy.spatial.transform import Rotation as R


class SpaceMouseFrankaControl:
    def __init__(self):
        rospy.init_node('quest_franka_control')

        # Params (tweak as needed)
        self.local_frame = rospy.get_param('~local_frame', False)      # True: q * dq ; False: dq * q

        self.pub = rospy.Publisher(
            '/cartesian_impedance_controller/equilibrium_pose',
            PoseStamped, queue_size=10
        )

        self.latest_state = None
        self.origin_state = None
        self.latest_quest_pose = None
        self.tare_quest_pose = None
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.state_cb, queue_size=1)
        rospy.Subscriber('/q2r_right_hand_pose', PoseStamped, self.quest_pose_cb, queue_size=1)
        rospy.Subscriber('/q2r_right_hand_inputs', OVR2ROSInputs, self.quest_button_cb, queue_size=1)
        
        self.is_grasped = False
        self.grasp_toggle = False
        self.gripper_client = SimpleActionClient("/franka_gripper/grasp", GraspAction)
        self.gripper_client.wait_for_server()

        rospy.loginfo("Quest → Franka equilibrium pose teleop ready.")

    def state_cb(self, msg: FrankaState):
        self.latest_state = msg
    
    def quest_pose_cb(self, msg: PoseStamped):
        self.latest_quest_pose = msg
        
    def quest_button_cb(self, msg: OVR2ROSInputs):
        if msg.press_middle > 0:
            if self.origin_state is None:
                self.origin_state = self.latest_state
            if self.tare_quest_pose is None:
                self.tare_quest_pose = self.latest_quest_pose
                
            # ---- Extract current pose from O_T_EE (column-major 4x4) ----
            T = np.array(self.origin_state.O_T_EE, dtype=float).reshape((4, 4), order='F')
            R_org = T[0:3, 0:3]                    # rotation matrix
            p_org = T[0:3, 3].copy()               # translation (last column)
                
            # add latest_quest_pose to origin_state and get p_new and q_new
            if self.latest_quest_pose is None:
                return

            # ---- Compute relative Quest pose wrt tare ----
            # Extract tare pose
            p_tare = np.array([
                self.tare_quest_pose.pose.position.x,
                self.tare_quest_pose.pose.position.y,
                self.tare_quest_pose.pose.position.z
            ])
            q_tare = np.array([
                self.tare_quest_pose.pose.orientation.x,
                self.tare_quest_pose.pose.orientation.y,
                self.tare_quest_pose.pose.orientation.z,
                self.tare_quest_pose.pose.orientation.w
            ])

            # Current Quest pose
            p_curr = np.array([
                self.latest_quest_pose.pose.position.x,
                self.latest_quest_pose.pose.position.y,
                self.latest_quest_pose.pose.position.z
            ])
            q_curr = np.array([
                self.latest_quest_pose.pose.orientation.x,
                self.latest_quest_pose.pose.orientation.y,
                self.latest_quest_pose.pose.orientation.z,
                self.latest_quest_pose.pose.orientation.w
            ])

            # Relative rotation (q_rel = q_tare^-1 * q_curr)
            q_rel = (R.from_quat(q_tare).inv() * R.from_quat(q_curr)).as_quat()

            # Relative translation (expressed in tare frame)
            p_rel = R.from_quat(q_tare).inv().apply(p_curr - p_tare)

            # These go into the robot combination
            p_q = p_rel
            q_q = q_rel

            q_org = R.from_matrix(R_org).as_quat()

            if self.local_frame:
                q_new = (R.from_quat(q_org) * R.from_quat(q_q)).as_quat()
            else:
                q_new = (R.from_quat(q_q) * R.from_quat(q_org)).as_quat()

            p_new = p_org + p_q * 2.0
            
            # ---- Publish PoseStamped in the same frame as the state ----
            cmd = PoseStamped()
            cmd.header.frame_id = self.latest_state.header.frame_id or "panda_link0"
            cmd.header.stamp = rospy.Time.now()
            cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = p_new.tolist()
            cmd.pose.orientation.x = float(q_new[0])
            cmd.pose.orientation.y = float(q_new[1])
            cmd.pose.orientation.z = float(q_new[2])
            cmd.pose.orientation.w = float(q_new[3])
            self.pub.publish(cmd)  
        else:
            self.origin_state = None
            self.tare_quest_pose = None
            
        if msg.press_index and not self.grasp_toggle:
            if self.is_grasped:
                grasp_goal = GraspGoal()
                grasp_goal.width = 0.08
                grasp_goal.epsilon.inner = 0.08
                grasp_goal.epsilon.outer = 0.08
                grasp_goal.speed = 0.1
                grasp_goal.force = 5.0
                self.gripper_client.send_goal(grasp_goal)
                self.is_grasped = False
            else:
                grasp_goal = GraspGoal()
                grasp_goal.width = 0.0
                grasp_goal.epsilon.inner = 0.08
                grasp_goal.epsilon.outer = 0.08
                grasp_goal.speed = 0.1
                grasp_goal.force = 5.0
                self.gripper_client.send_goal(grasp_goal)
                self.is_grasped = True
            self.grasp_toggle = True
        if not msg.press_index:
            self.grasp_toggle = False        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        SpaceMouseFrankaControl().run()
    except rospy.ROSInterruptException:
        pass
