#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
from vision_manip_pipeline.srv import VisionManip

def vision_manip_client(obj_name):
    rospy.wait_for_service('vision_manip')
    try:
        vision_manip_server = rospy.ServiceProxy('vision_manip', VisionManip)
        resp = vision_manip_server(obj_name)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed %s"%e


if __name__ == "__main__":
    if len(sys.argv) == 2:
        obj_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting : %s"%(obj_name)
    print "%s"%(vision_manip_client(obj_name))
