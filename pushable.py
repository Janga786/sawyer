#!/usr/bin/env python

import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty

import intera_interface
from intera_interface import CHECK_VERSION

class JointSprings(object):
    """
    Virtual Joint Springs class for torque example.
    """
    def __init__(self, reconfig_server, limb="right"):
        self._dyn = reconfig_server
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0
        self._limb = intera_interface.Limb(limb)
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = 19
            self._damping[joint] = 2

    def _update_forces(self):
        # get latest spring constants
        self._update_parameters()
        # disable cuff interaction
        self._pub_cuff_disable.publish()
        # create our command dict
        cmd = dict()
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        for joint in self._start_angles.keys():
            # Hooke's law and damping
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] - cur_pos[joint])
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        self._limb.move_to_neutral()

    def attach_springs(self):
        self._start_angles = self._limb.joint_angles()
        control_rate = rospy.Rate(self._rate)
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet control rate timeout.")
                break
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

def main():
    """RSDK Joint Torque Example: Joint Springs"""
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message("Cannot detect any limb parameters on this robot. Exiting.", "ERROR")
        return

    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()
    # Parse Input Arguments
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help='limb on which to attach joint springs'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Dynamic reconfigure config
    config_name = ''.join([robot_name, "JointSpringsExampleConfig"])
    config_module = "intera_examples.cfg"
    cfg = importlib.import_module('.'.join([config_module, config_name]))

    # Start ROS node
    print("Initializing node... ")
    rospy.init_node("sdk_joint_torque_springs_{0}".format(args.limb))
    dynamic_cfg_srv = Server(cfg, lambda config, level: config)
    js = JointSprings(dynamic_cfg_srv, limb=args.limb)
    rospy.on_shutdown(js.clean_shutdown)
    js.move_to_neutral()
    js.attach_springs()

if __name__ == "__main__":
    main()

