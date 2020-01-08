#!/usr/bin/env python
import rospy
import sys
from gpd.msg import GraspConfigList
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from vision_manip_pipeline.srv import GetObjLoc

def get_obj_loc_client(obj_name):
    rospy.wait_for_service('get_object_loc')
    try:
        get_obj_loc = rospy.ServiceProxy('get_object_loc', GetObjLoc)
        resp1 = get_obj_loc(obj_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [obj_name]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        obj_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting: %s"%(obj_name)
    print "%s"%(get_obj_loc_client(obj_name))