#!/usr/bin/env python

import sys
import rospy
from table_task_sim.srv import *
from geometry_msgs.msg import Pose

def pick_client():
    rospy.wait_for_service('pick_service')
    rospy.wait_for_service('place_service')
    try:
        pick = rospy.ServiceProxy('pick_service', PickUpObject)
        place = rospy.ServiceProxy('place_service', PlaceObject)
        p = Pose()
        p.position.x = -0.25
        p.position.y = -0.25
        p.position.z = 0
        resp1 = pick(1, "Left_Bread")
        resp1 = place(1, p)
        resp1 = pick(1, "Lettuce")
        p.position.x = -0.15
        p.position.y = -0.15
        p.position.z = 0
        resp1 = place(1, p)

        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "result = %s"%(pick_client())
