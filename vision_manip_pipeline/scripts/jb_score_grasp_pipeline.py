#!/usr/bin/env python
import rospy
import sys
import numpy as np
import cPickle as pickle
import tf
from jb_yolo_obj_det_client import *
from jb_get_grasp_client import *
from jb_conv_coord_client import *
from jb_pub_workspace_corners_client import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gpd.msg import GraspConfig
import roslaunch

import moveit_commander
import moveit_msgs.msg
import roslib

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)



#====================================================

def main(obj_name):


    # Create a ROS node.
    rospy.init_node('vision_manip')
    # rospy.on_shutdown(self.shutdown)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/janelle/onr_ws/src/gpd/launch/jb_tutorial1.launch"])

    # rosservice call with object to find location of in YOLO
        # So get the bounding box of the image and 
        # calculate the center of it as a start for the grasping window?
    resp = get_obj_loc_client(obj_name)
    print resp
    x = (resp.xmax - resp.xmin)/2 + resp.xmin
    y = (resp.ymax - resp.ymin)/2 + resp.ymin
    print x,y

    # if object not detected, then exit?
    if x == 0 and y == 0:
        print "Error: Object not detected, try again!"
        print "Score is: {}".format(0)
        return 0

    # calculate the transform of the point in the raw image to
    # the grasping window in the depth point cloud!
    resp3 = conv_coord_client(x,y)    
    print resp3

    #  first set the param for the workspace based on the response?!?
    eps = 0.5
    cube = [resp3.newX - eps, resp3.newX + eps, resp3.newY - eps, resp3.newY + eps, resp3.newZ - eps, resp3.newZ + eps]
    # cube = [1,1.1,1,1.1,1,1.1]
    print "cube to search for graps:"
    print cube
    if rospy.has_param("/detect_grasps/"):
        rospy.delete_param("/detect_grasps/")
    rospy.set_param('/detect_grasps/workspace', cube)
    rospy.set_param('/detect_grasps/workspace_grasps', cube)

    # relaunch the grasp stuffsssss
    launch.start()

    # # rosservice call to gpd with the calculated grasping window in the 
    # # point cloud to get the top grasp 
    resp2 = get_grasp_client(resp3.newX, resp3.newY, resp3.newZ)
    print resp2
    print "Score is: {}".format(resp2.grasp.score.data)
    return resp2.grasp.score.data

    launch.shutdown()

    # IF GRASP NOT FOUND, return 0?
    if resp2 == None:
    # if resp2.grasp.score == 0:
        print "Error: No grasp found, will now return"
        print "Score is: {}".format(0)
        return 0



# ==================== MAIN ====================
if __name__ == '__main__':


    if len(sys.argv) == 2:
        obj_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting: %s"%(obj_name)

    main(obj_name)

