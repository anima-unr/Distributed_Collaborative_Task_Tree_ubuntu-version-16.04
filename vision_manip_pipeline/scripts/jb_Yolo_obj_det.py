#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

# global variable to store object_locations
obj_loc = []


# Callback function to receive bounding boxes.
def callback(msg):
    global obj_loc
    obj_loc = msg.boundingBoxes



def talker(obj_name):
    # Create a ROS node.
    rospy.init_node('get_object_loc')

    # Subscribe to the ROS topic that contains the grasps.
    sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    # Wait for grasps to arrive.
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():    
        # print obj_loc

        for item in obj_loc:
            if item.Class == obj_name:
                print item

                # calculate the center of the bounding box
                x = (item.xmax - item.xmin)/2 + item.xmin
                y = (item.ymax - item.ymin)/2 + item.ymin

                print x,y 

        rate.sleep()

# ==================== MAIN ====================
if __name__ == '__main__':
    talker('cup')