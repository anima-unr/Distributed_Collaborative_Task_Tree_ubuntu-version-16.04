#!/usr/bin/env python
import rospy
import sys
from vision_manip_pipeline.srv import Conv2DTo3D

def conv_coord_client(x,y):
    rospy.wait_for_service('conv_coord')
    try:
        conv_coord = rospy.ServiceProxy('conv_coord', Conv2DTo3D)
        resp1 = conv_coord(x,y)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s,%s [x,y]"%(sys.argv[0],sys.argv[1])

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting: %d,%d"%(x,y)
    print "%s"%(conv_coord_client(x,y))
