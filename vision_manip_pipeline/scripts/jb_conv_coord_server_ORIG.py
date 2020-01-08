#!/usr/bin/env python
import rospy
import numpy as np
import tf
from vision_manip_pipeline.srv import Conv2DTo3D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# global variable to store grasps
img = []

# get depth image depth at coord (x_d, y_d)
def depth(x,y):
    global img

    # TODO_PR2_TOPIC_CHANGE
    # rospy.init_node("depth_convert")
    rospy.Subscriber("/camera/depth_registered/image_raw", Image, callback)
    # rospy.Subscriber("/kinect_head/depth_registered/image_raw", Image, callback)

    while img == []:
        pass

    print x,y
    # print img[y][x]
    # depth = img[y][x]/1000
    depth = img[x][y]/1000

    # print img
    print 'depth is: {}'.format(depth)

    return depth

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global img
    # print data.height, data.width

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="16UC1")
        depth_array = np.array(cv_image) # convert to np array
        depth_array = depth_array.astype(float) # convert np array from int to float

        # pickle.dump(data, open('data.txt', "w"))

        # # print data.encoding
        # print cv_image
        # print depth_array

        # # print np.count_nonzero(~np.isnan(cv_image))
        # print "total values in image: {}".format( depth_array.shape[0]*depth_array.shape[1])
        # print "total zero values in image: {}".format(depth_array.size - np.count_nonzero(depth_array))
        # print "total nan values in image: {}".format(np.count_nonzero(np.isnan(depth_array)))
        # print "total nonzero values in image: {}".format(np.count_nonzero(depth_array))
    except CvBridgeError, e:
        print e

    img = depth_array
    rospy.sleep(1)

# convert from depth image to 3d metric space...?
def convert2Dto3D(x_d,y_d):

    # from: http://nicolas.burrus.name/index.php/Research/KinectCalibration
    fx_d = 5.9421434211923247e+02
    fy_d = 5.9104053696870778e+02
    cx_d = 3.3930780975300314e+02
    cy_d = 2.4273913761751615e+02

    # from: http://nicolas.burrus.name/index.php/Research/KinectCalibration
    x = (x_d - cx_d) * depth(x_d,y_d) / fx_d
    y = (y_d - cy_d) * depth(x_d,y_d) / fy_d
    z = depth(x_d,y_d)

    return (x,y,z)


# Handle of service
def handle_conv_coord(req):

    # calculate the transform of the point in the raw image to
    # the grasping window in the depth point cloud!
    # TODO! Because this is hard!!!!! T.T

    x,y,z = convert2Dto3D(req.y,req.x)
    print x,y,z

    # convert from /camera_rgb_optical_frame to /camera_depth_optical_frame
    now = rospy.Time(0)
    listener = tf.TransformListener(True, rospy.Duration(10.0))
    listener.waitForTransform('/camera_depth_optical_frame', '/camera_rgb_optical_frame', now, rospy.Duration(3.0))
    (trans, rot) = listener.lookupTransform( '/camera_depth_optical_frame', '/camera_rgb_optical_frame', now)
    # listener.waitForTransform('/camera_rgb_optical_frame', '/camera_depth_optical_frame', now, rospy.Duration(3.0))
    # (trans, rot) = listener.lookupTransform( '/camera_rgb_optical_frame', '/camera_depth_optical_frame', now)

    print trans,rot

    mat = tf.TransformerROS()
    rotMat = mat.fromTranslationRotation(trans,rot)

    # print rotMat
    pnt = np.asarray([x,y,z,1]).reshape(4,1)
    # print pnt
    # print np.matrix(rotMat)*pnt
    newPnt = rotMat.dot(pnt)
    newX = (newPnt[0]/newPnt[3])[0]
    newY = (newPnt[1]/newPnt[3])[0]
    newZ = (newPnt[2]/newPnt[3])[0]

    # print newX,newY,newZ
    print "[{},{},{}]".format(newX,newY,newZ)
    print (newX, newY, newZ)
    return (newX, newY, newZ)

# Server set up
def conv_coord_server():
    # Create a ROS node.
    rospy.init_node('conv_coord_server')

    # Create a service
    s = rospy.Service('conv_coord', Conv2DTo3D, handle_conv_coord)

    # spin service   
    print "Ready to convert coords from 2D to 3D!"
    rospy.spin()

#------------------------------------------------
if __name__ == "__main__":
    conv_coord_server()

