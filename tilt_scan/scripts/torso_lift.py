#! /usr/bin/env python
from __future__ import print_function
import time
import rospy
from pr2_controllers_msgs.msg import *

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

def torso_lift():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = pr2_controllers_msgs.msg.SingleJointPositionGoal(0.2, rospy.Duration(2.0), 1.0)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    print("torso lift completed: ready to go!")

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('torso_lift')

        # wait a few seconds for the rest of the nodes to start
        #time.sleep(5)

        result = torso_lift()
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
