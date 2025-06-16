import rospy
import intera_interface
import intera_external_devices


def complex_interaction():
    """Performs a complex sequence of motions with Sawyer."""
    # Initialize ROS node
    rospy.init_node('complex_interaction')

    limb = intera_interface.Limb('right')
    gripper = intera_interface.Gripper('right_gripper')

    # Define joint positions for initial, handshake, pick and place
    neutral = {'right_j0': 0.0,
               'right_j1': -1.0,
               'right_j2': 0.0,
               'right_j3': 1.5,
               'right_j4': 0.0,
               'right_j5': 0.5,
               'right_j6': 0.0}

    handshake = {'right_j0': 0.3,
                 'right_j1': -0.8,
                 'right_j2': 0.0,
                 'right_j3': 1.2,
                 'right_j4': 0.0,
                 'right_j5': 1.0,
                 'right_j6': 0.3}

    pick = {'right_j0': 0.5,
            'right_j1': -1.0,
            'right_j2': 0.2,
            'right_j3': 1.8,
            'right_j4': 0.1,
            'right_j5': 1.2,
            'right_j6': 0.5}

    place = {'right_j0': -0.5,
             'right_j1': -0.8,
             'right_j2': -0.2,
             'right_j3': 1.6,
             'right_j4': 0.1,
             'right_j5': 1.3,
             'right_j6': -0.5}

    # Move to neutral
    limb.move_to_joint_positions(neutral)

    # Wave arm for greeting
    for _ in range(2):
        handshake['right_j6'] = 0.5
        limb.move_to_joint_positions(handshake)
        handshake['right_j6'] = 0.1
        limb.move_to_joint_positions(handshake)

    # Pick up object
    limb.move_to_joint_positions(pick)
    gripper.close()
    rospy.sleep(1.0)

    # Place object
    limb.move_to_joint_positions(place)
    gripper.open()
    rospy.sleep(1.0)

    # Return to neutral
    limb.move_to_joint_positions(neutral)


if __name__ == '__main__':
    complex_interaction()
