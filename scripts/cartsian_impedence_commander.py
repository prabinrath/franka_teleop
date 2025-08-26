#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from scipy.spatial.transform import Rotation as R

class SpaceMouseFrankaControl:
    def __init__(self):
        rospy.init_node('spacemouse_franka_control')

        # Params (tweak as needed)
        self.lin_scale = rospy.get_param('~lin_scale', 0.1)         # m per axis unit
        self.rot_scale_deg = rospy.get_param('~rot_scale_deg', 15.0)   # deg per axis unit
        self.deadband = rospy.get_param('~deadband', 1e-3)            # axis deadband
        self.local_frame = rospy.get_param('~local_frame', False)      # True: q * dq ; False: dq * q

        self.pub = rospy.Publisher(
            '/cartesian_impedance_controller/equilibrium_pose',
            PoseStamped, queue_size=10
        )

        self.latest_state = None
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.state_cb, queue_size=1)
        rospy.Subscriber('/spacenav/joy', Joy, self.joy_cb, queue_size=1)

        rospy.loginfo("SpaceMouse → Franka equilibrium pose teleop ready.")

    def state_cb(self, msg: FrankaState):
        self.latest_state = msg

    def joy_cb(self, joy: Joy):
        if self.latest_state is None:
            rospy.logwarn_throttle(2.0, "No Franka state yet; ignoring SpaceMouse input.")
            return

        axes = list(joy.axes) if joy.axes else [0.0]*6
        if len(axes) < 6:
            axes += [0.0]*(6 - len(axes))

        # Deadband
        axes = [a if abs(a) >= self.deadband else 0.0 for a in axes]
        if all(abs(a) < 1e-9 for a in axes):
            return  # nothing to do

        # ---- Extract current pose from O_T_EE (column-major 4x4) ----
        T = np.array(self.latest_state.O_T_EE, dtype=float).reshape((4, 4), order='F')
        R_curr = T[0:3, 0:3]                    # rotation matrix
        p_curr = T[0:3, 3].copy()               # translation (last column)

        rot_curr = R.from_matrix(R_curr)

        # ---- Build delta from SpaceMouse: axes[0:3]=xyz, [3:6]=rpy ----
        dx, dy, dz = axes[0:3]
        droll, dpitch, dyaw = axes[3:6]

        p_new = p_curr + np.array([dx, dy, dz]) * self.lin_scale

        rot_scale = np.deg2rad(self.rot_scale_deg)
        d_rpy = np.array([droll, dpitch, dyaw]) * rot_scale

        # intrinsic xyz (local frame) like your original scipy use
        d_rot = R.from_euler('xyz', d_rpy)

        if self.local_frame:
            rot_new = rot_curr * d_rot           # apply delta in body frame
        else:
            rot_new = d_rot * rot_curr           # apply delta in world frame

        q_new = rot_new.as_quat()                # [x, y, z, w]

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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        SpaceMouseFrankaControl().run()
    except rospy.ROSInterruptException:
        pass
