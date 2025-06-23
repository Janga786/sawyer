#!/usr/bin/env python

import rospy
from intera_interface import Limb, Gripper
import intera_interface

def main():
    rospy.init_node('custom_demo')
    limb = Limb('right')
    gripper = Gripper('right')

    # 1. WAVE: Move arm up, then left/right
    print("Waving hello...")
    wave_pose = limb.joint_angles()
    # Raise the elbow and shoulder
    wave_pose['right_e0'] = 0.5
    wave_pose['right_s1'] = 1.0
    limb.move_to_joint_positions(wave_pose)
    for i in range(3):
        wave_pose['right_w1'] = 1.0
        limb.move_to_joint_positions(wave_pose)
        rospy.sleep(0.4)
        wave_pose['right_w1'] = -1.0
        limb.move_to_joint_positions(wave_pose)
        rospy.sleep(0.4)

    # 2. POINT: Extend arm forward, gripper open
    print("Pointing at object...")
    point_pose = wave_pose.copy()
    point_pose['right_s0'] = 0.0
    point_pose['right_s1'] = 0.6
    point_pose['right_e1'] = 1.5
    point_pose['right_w0'] = 0.0
    point_pose['right_w1'] = 0.0
    point_pose['right_w2'] = 0.0
    limb.move_to_joint_positions(point_pose)
    gripper.open()
    rospy.sleep(1.0)

    # 3. FIST BUMP: Move close to camera (assume camera in front)
    print("Fist bump!")
    fist_pose = point_pose.copy()
    fist_pose['right_s1'] = 1.2
    fist_pose['right_e1'] = 2.0
    limb.move_to_joint_positions(fist_pose)
    gripper.close()
    rospy.sleep(1.0)

    print("Done!")

if __name__ == '__main__':
    main()
