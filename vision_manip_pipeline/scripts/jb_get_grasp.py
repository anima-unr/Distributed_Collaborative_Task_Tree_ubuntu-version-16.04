#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList

# global variable to store grasps
grasps = []


# Callback function to receive grasps.
def callback(msg):
    global grasps
    grasps = msg.grasps

def talker():
    # Create a ROS node.
    rospy.init_node('get_grasps')

    # Subscribe to the ROS topic that contains the grasps.
    sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

    # Wait for grasps to arrive.
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():    
        print '.'
        if len(grasps) > 0:
            rospy.loginfo('Received %d grasps.', len(grasps))

            # get the best grasp
            rospy.loginfo('Top grasp was:')
            top_grasp = grasps[0];
            for grasp in grasps:
                # print grasp
                # print '\n'

                # what to do here..... what qualifies as best?.... 
                #  is it within epsilon of x-y location from image?
                #  becuase how should I handle that position then?
                #  maybe turn this into a rosservice which will have 
                #  a main script which detecs to object we want
                #  which will have to have a rosserivce to YOLO to 
                #  get the location of a given object?
                #  and then given that location makes a service call to 
                #  this function which will then return the best grasp
                #  near that location? 
                #  then will have to convert grasp to a moveit acceptable
                #  format in the main script and then use moveit to move
                #  to that grasp location and orientation using the pr2
                #  in rviz to test everything?!?!
                    # or better yet, get location of object first, then 
                    # service call to grasp stuff which just returns grasps
                    # in a small workspace around that object, and then call
                    # to this script to return the best grasp out of all grasps 
                    # which will just be top score since will already have 
                    # segmented it out in space??

                if grasp.score > top_grasp.score:
                    top_grasp = grasp
            

            print top_grasp
            break
        rate.sleep()


# ==================== MAIN ====================
if __name__ == '__main__':
    talker()