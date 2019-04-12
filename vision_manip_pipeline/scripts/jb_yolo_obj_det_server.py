#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from vision_manip_pipeline.srv import GetObjLoc

# global variable to store object_locations
obj_loc = []


# Callback function to receive bounding boxes.
def callback(msg):
    global obj_loc
    obj_loc = msg.bounding_boxes

# Handle of service
def handle_get_obj_loc(req):

    # Subscribe to the ROS topic that contains the grasps.
    sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    # print req

    # wait until the subscriber gets location objects from topic
    while obj_loc == []:
        #print '.'
        pass

    if req.obj_name == "teddy_bear":
        req.obj_name = "teddy bear"

    if req.obj_name == "sports_ball":
        req.obj_name = "sports ball"

    #  get the location of the requested object
    # - if multiple objects, return one with highest probability!
    top_item = BoundingBox(probability=0.0);
    for item in obj_loc:
        if item.Class == req.obj_name:
            if item.probability > top_item.probability:
                top_item = item

                # TODO: move this logic into main script for now!!!
                # calculate the center of the bounding box
                # top_item.x = (item.xmax - item.xmin)/2 + item.xmin
                # top_item.y = (item.ymax - item.ymin)/2 + item.ymin

    print "Returning %s with highest probability [%s]"%(req.obj_name, top_item)
    return [top_item.Class, top_item.probability, top_item.xmin, top_item.ymin, top_item.xmax, top_item.ymax]

# Server set up
def get_obj_loc_server():
    # Create a ROS node.
    rospy.init_node('get_object_loc_server')

    # Create a service
    s = rospy.Service('get_object_loc', GetObjLoc, handle_get_obj_loc)

    # spin service
    print "Ready to get locs!"
    rospy.spin()

#------------------------------------------------
if __name__ == "__main__":
    get_obj_loc_server()
