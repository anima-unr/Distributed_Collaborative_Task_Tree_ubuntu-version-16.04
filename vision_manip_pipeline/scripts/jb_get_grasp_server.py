#!/usr/bin/env python
import rospy
from gpd.msg import GraspConfigList
from gpd.msg import GraspConfig
from vision_manip_pipeline.srv import GetGrasp

# global variable to store grasps
grasps = []

# Callback function to receive grasps.
def callback(msg):
    global grasps
    grasps = msg.grasps
    print grasps

# Handle of service
def handle_get_grasp(req):
    global grasps

    # set a timer here to keep track of wait time....?
    start = rospy.get_time()
    lapse = 0
    threshold = 10

    # keep waiting until get grasp with close location......
    graspList = GraspConfigList()

    notFound = True
    while notFound:

        print req

        cube = rospy.get_param('/detect_grasps/workspace')

        # wait until the subscriber gets location objects from topic
        top_grasp = GraspConfig()
        top_grasp.score.data = 0
        while grasps == []:
            current = rospy.get_time()
            lapse = current - start
            if lapse % 1 == 0:
                print lapse
            if lapse > threshold:
                print "ERROR: Took too long to get grasp, will now force exit."
                # return top_grasp
                return (graspList, len(graspList.grasps))

        # search for a grasp close to the object location    
        # rospy.loginfo('Top grasp was:')
        top_grasp = GraspConfig()
        top_grasp.score.data = 0
        print grasps
        for grasp in grasps:

            # not tested yet, but if can't launch inside from roslaunch api becuase can't set parms in launch file, then test something like this to get the graps at the specified location!
            # if abs( req.x - grasp.surface.x) < eps and abs( req.y - grasp.surface.y) < eps: 
            # if req.x > cube[0] and req.x < cube[1] and req.y > cube[2] and req.y < cube[3] and req.z > cube[4] and req.z < cube[5]: 
                if grasp.score > top_grasp.score:
                    top_grasp = grasp
                    notFound = False
                    print "GRASP FOUND!"

                graspList.grasps.append(grasp)
                notFound = False


        # otherwise no close grasps found yet, so reset grasps and try again!
        if notFound:
            print "ERRRORRRRR NO CLOSE GRASP FOUND!!!!!! Trying again! \n\n\n"
            # grasps = []
            top_grasp = None
            # rospy.sleep(0.1)
            current = rospy.get_time()
            lapse = current - start
            print lapse
            if lapse > threshold:
                print "ERROR: Took too long to get CLOSE grasp, will now force exit."
                # return top_grasp
                return (graspList, len(graspList.grasps))

    # grasp was found, return it!
    # print "Returning grasp with highest score [%s]"%(top_grasp)
    grasps = [] # need to clear this before exiting, but can't becuase used before assigned
    # return top_grasp

    return (graspList, len(graspList.grasps))

# Server set up
def get_grasp_server():
    # Create a ROS node.
    rospy.init_node('get_grasp_server')

    # Subscribe to the ROS topic that contains the grasps.
    sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback, queue_size = 1)

    # Create a service
    s = rospy.Service('get_grasp', GetGrasp, handle_get_grasp)

    # spin service 
    rate = rospy.Rate(0.5)  
    while not rospy.is_shutdown():
        print "Ready to get top grasp!"
        sub.unregister()
        # pot hole with a roadsign - TODO
        sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback, queue_size = 1)
        rate.sleep()

    # # pot hole with a roadsign - TODO
    # sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback, queue_size = 1)
    # rospy.spin()

#------------------------------------------------
if __name__ == "__main__":
    get_grasp_server()

