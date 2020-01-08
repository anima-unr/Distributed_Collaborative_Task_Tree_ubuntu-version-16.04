#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from vision_manip_pipeline.srv import GetObjLoc

# Handle of service
def handle_grasp_object(req):

    # TODO: Finish this server for grasp object which is basically just the main from jb_vision_manip_pipeline.py
    #  Then need to write script for baxter in gazebo to call this service to get location to move to object and then write moviet code after return to move arm!!!!

    # rosservice call with object to find location of in YOLO
        # So get the bounding box of the image and 
        # calculate the center of it as a start for the grasping window?
    resp = get_obj_loc_client(req.obj_name)
    print resp
    x = (resp.xmax - resp.xmin)/2 + resp.xmin
    y = (resp.ymax - resp.ymin)/2 + resp.ymin
    print x,y

    # calculate the transform of the point in the raw image to
    # the grasping window in the depth point cloud!
    resp3 = conv_coord_client(x,y)    
    print resp3

    # # rosservice call to gpd with the calculated grasping window in the 
    # # point cloud to get the top grasp 
    resp2 = get_grasp_client(resp3.newX, resp3.newY, resp3.newZ)
    print resp2

    return resp2

# Server set up
def get_obj_loc_server():
    # Create a ROS node.
    rospy.init_node('grasp_object_server')

    # Create a service
    s = rospy.Service('grasp_object', GraspObj, handle_grasp_object)

    # spin service   
    print "Ready to grasp objects!"
    rospy.spin()

#------------------------------------------------
if __name__ == "__main__":
    get_obj_loc_server()

