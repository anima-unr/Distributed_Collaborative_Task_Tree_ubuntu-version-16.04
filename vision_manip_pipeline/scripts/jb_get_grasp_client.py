#!/usr/bin/env python
import rospy
import sys
from gpd.msg import GraspConfigList
from vision_manip_pipeline.srv import GetGrasp

def get_grasp_client(x,y,z):
    rospy.wait_for_service('get_grasp')
    try:
        get_grasp = rospy.ServiceProxy('get_grasp', GetGrasp)
        resp1 = get_grasp(x,y,z)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s,%s [x,y]"%(sys.argv[0],sys.argv[1])

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        z = int(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting: %d,%d"%(x,y,z)
    print "%s"%(get_grasp_client(x,y,z))

