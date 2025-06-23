#!/usr/bin/env python3
"""
Simple Sawyer “Show‐Off” Demo

1. Enable Sawyer
2. Pan head left → right → center
3. Draw a 0.2 m square in the XY plane
4. Wave the wrist 3 times
5. Do a quick “dance” (small jitters)
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped
from tf_conversions import posemath
import PyKDL

import intera_interface
from intera_interface import CHECK_VERSION
from intera_motion_interface import MotionTrajectory, MotionWaypoint, MotionWaypointOptions
import intera_motion_msgs.msg


class SawyerDemo:
    def __init__(self):
        rospy.init_node('simple_showoff_demo')

        # Interfaces
        self._limb    = intera_interface.Limb()
        self._head    = intera_interface.Head()
        self._gripper = intera_interface.Gripper('right_gripper')
        self._rs      = intera_interface.RobotEnable(CHECK_VERSION)

        # Enable robot
        rospy.loginfo("Enabling robot…")
        self._rs.enable()
        rospy.on_shutdown(self._rs.disable)
        rospy.sleep(0.5)

    def _move_to_pose(self, pose, speed=0.3, accel=0.3, timeout=10.0):
        opts = intera_motion_msgs.msg.TrajectoryOptions()
        opts.interpolation_type = opts.CARTESIAN
        traj = MotionTrajectory(trajectory_options=opts, limb=self._limb)

        wp_opts = MotionWaypointOptions(
            max_linear_speed=speed,
            max_linear_accel=accel,
            max_rotational_speed=1.0,
            max_rotational_accel=1.0
        )
        wp = MotionWaypoint(options=wp_opts.to_msg(), limb=self._limb)
        wp.set_cartesian_pose(pose, 'right_hand')
        traj.append_waypoint(wp.to_msg())
        result = traj.send_trajectory(timeout=timeout)
        if not (result and result.result):
            raise RuntimeError("Move failed")

    def _move_relative(self, dx, dy, dz, reference='tip'):
        current = self._limb.tip_state('right_hand').pose
        frame   = posemath.fromMsg(current)
        delta   = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0),
                              PyKDL.Vector(dx, dy, dz))
        target  = frame * delta if reference=='tip' else delta * frame
        ps      = PoseStamped()
        ps.pose = posemath.toMsg(target)
        self._move_to_pose(ps)

    def head_sweep(self):
        for ang in [-0.8, 0.8, 0.0]:
            rospy.loginfo(f"Head pan → {ang:.2f} rad")
            self._head.set_pan(ang, speed=0.8, timeout=2.0)
            rospy.sleep(0.6)

    def draw_square(self, side=0.2):
        rospy.loginfo("Drawing square")
        # start at current pose
        for dx, dy in [( side,  0),
                       ( 0,  side),
                       (-side, 0),
                       ( 0, -side)]:
            self._move_relative(dx, dy, 0)

    def wave(self, times=3):
        rospy.loginfo("Waving")
        for _ in range(times):
            self._limb.set_joint_position_speed_ratio(0.8)
            # small yaw around wrist ≈ joint_6
            self._limb.set_joint_position('right_j6',  0.5)
            rospy.sleep(0.4)
            self._limb.set_joint_position('right_j6', -0.5)
            rospy.sleep(0.4)
        # return to center
        self._limb.set_joint_position('right_j6', 0.0)
        rospy.sleep(0.4)

    def dance(self):
        rospy.loginfo("Dance time!")
        for dx, dy, dz in [(0.05,0,0),(-0.05,0,0),
                           (0,0.05,0),(0,-0.05,0),
                           (0,0,0.05),(0,0,-0.05)] * 2:
            self._move_relative(dx, dy, dz)
            rospy.sleep(0.2)

    def run(self):
        try:
            self.head_sweep()
            self.draw_square()
            self.wave()
            self.dance()
            rospy.loginfo("Demo complete!")
        except Exception as e:
            rospy.logerr(f"Demo failed: {e}")
            raise


if __name__ == '__main__':
    SawyerDemo().run()

