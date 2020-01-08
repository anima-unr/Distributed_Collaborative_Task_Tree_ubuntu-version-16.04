#!/usr/bin/env python
# import rospy
# import numpy as np
# import tf
# from sensor_msgs.msg import PointCloud2 as pc2

# import sensor_msgs.point_cloud2 as pc2_py

# from vision_manip_pipeline.srv import Conv2DTo3D
# # from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# # global variable to store grasps
# cloud = []
# PC2_data = []

# # get depth image depth at coord (x_d, y_d)
# def depth(x_d,y_d):
#     global cloud
#     global PC2_data

#     # rospy.init_node("depth_convert")
#     rospy.Subscriber("/camera/depth_registered/points", pc2, callback)

#     while PC2_data == [] or cloud == []:
#         # print '.'
#         pass

#     print PC2_data.width

#     # # convert PC2 data to pc2_py data version format
#     # cloud = PC2_data.data
#     # gen = pc2_py.read_points(cloud, skip_nans=True, field_names=("x","y","z"))

#     # extract points?
#     i = x_d + y_d*PC2_data.width 

#     print x_d
#     print y_d
#     print i
#     print len(cloud)


#     # eps = 100;
#     # for p in cloud:
#     #     if np.abs(p[0] - x_d) < eps and np.abs(p[1] - y_d) < eps:
#     #         print p




#     x = cloud[i][0]
#     y = cloud[i][1]
#     z = cloud[i][2]


#     return [x,y,z]

# def callback(data):
#     global PC2_data
#     global cloud

#     #  get data from pc2
#     PC2_data = data

#     # convert PC2 data to pc2_py data version format
#     gen = pc2_py.read_points(PC2_data, skip_nans=False, field_names=("x","y","z","r","g","b"))

#     cloud = []
#     for p in gen:
#         cloud.append(p)

#     rospy.sleep(1)


# # convert from depth image to 3d metric space...?
# def convert2Dto3D(x_d,y_d):

#     global cloud

#     # # from: http://nicolas.burrus.name/index.php/Research/KinectCalibration
#     # fx_d = 5.9421434211923247e+02
#     # fy_d = 5.9104053696870778e+02
#     # cx_d = 3.3930780975300314e+02
#     # cy_d = 2.4273913761751615e+02

#     # # from: http://nicolas.burrus.name/index.php/Research/KinectCalibration
#     # x = (x_d - cx_d) * depth(x_d,y_d) / fx_d
#     # y = (y_d - cy_d) * depth(x_d,y_d) / fy_d
#     # z = depth(x_d,y_d)

#     (x,y,z) = depth(x_d, y_d)


#     return (x,y,z)


# # Handle of service
# def handle_conv_coord(req):

#     # calculate the transform of the point in the raw image to
#     # the grasping window in the depth point cloud!
#     # TODO! Because this is hard!!!!! T.T

#     x,y,z = convert2Dto3D(req.y,req.x)
#     # print x,y,z
#     newX = x; newY = y; newZ = z;

#     # # convert from /camera_rgb_optical_frame to /camera_depth_optical_frame
#     # now = rospy.Time(0)
#     # listener = tf.TransformListener(True, rospy.Duration(10.0))
#     # listener.waitForTransform('/camera_depth_optical_frame', '/camera_rgb_optical_frame', now, rospy.Duration(3.0))
#     # (trans, rot) = listener.lookupTransform( '/camera_depth_optical_frame', '/camera_rgb_optical_frame', now)
#     # # listener.waitForTransform('/camera_rgb_optical_frame', '/camera_depth_optical_frame', now, rospy.Duration(3.0))
#     # # (trans, rot) = listener.lookupTransform( '/camera_rgb_optical_frame', '/camera_depth_optical_frame', now)

#     # print trans,rot

#     # mat = tf.TransformerROS()
#     # rotMat = mat.fromTranslationRotation(trans,rot)

#     # # print rotMat
#     # pnt = np.asarray([x,y,z,1]).reshape(4,1)
#     # # print pnt
#     # # print np.matrix(rotMat)*pnt
#     # newPnt = rotMat.dot(pnt)
#     # newX = (newPnt[0]/newPnt[3])[0]
#     # newY = (newPnt[1]/newPnt[3])[0]
#     # newZ = (newPnt[2]/newPnt[3])[0]

#     # print newX,newY,newZ
#     print "[{},{},{}]".format(newX,newY,newZ)
#     print (newX, newY, newZ)
#     return (newX, newY, newZ)



# # Server set up
# def conv_coord_server():
#     # Create a ROS node.
#     rospy.init_node('conv_coord_server')

#     # Create a service
#     s = rospy.Service('conv_coord', Conv2DTo3D, handle_conv_coord)

#     # spin service   
#     print "Ready to convert coords from 2D to 3D!"
#     rospy.spin()

# #------------------------------------------------
# if __name__ == "__main__":
#     conv_coord_server()


#!/usr/bin/env python
import rospy
import numpy as np
import tf
from vision_manip_pipeline.srv import Conv2DTo3D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *

# global variable to store grasps
img = []
listener = 0

# get depth image depth at coord (x_d, y_d)
def depth(x,y):
    global img

    # TODO_PR2_TOPIC_CHANGE
    # rospy.Subscriber("/camera/depth_registered/image_raw", Image, callback)
    # rospy.Subscriber("/kinect_head/depth_registered/image_raw", Image, callback)
    rospy.Subscriber("/local/depth_registered/image_raw", Image, callback)

    while img == []:
        pass

    print x,y
    # print img[y][x]
    # depth = img[y][x]/1000
    #  TODO: Fix this logic so it checks a square around the x,y instead? 
    #       but might get overwritten if we use the C version of this server!!! 
    #       This shouldn't matter becuase the math will change completely in the C version!
    while img[x][y] == 0:
        x = x+1;
        y = y-1;
        # print img[x][y]
    print "modified x,y, since image point was 0: {},{}".format(x,y)
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
# def convert2Dto3D(x_d,y_d):
def convert2Dto3D(y_d,x_d):

    # from camera_info topic K matrix
    # fx_d = 516.6948358950754
    # fy_d = 515.9583492364502
    # cx_d = 313.6291066465527
    # cy_d = 245.1962086034237
    
    # from camera_info topic P matrix
    # FROM THE OFFSIDE KINECT
    fx_d = 513.3190307617188
    fy_d = 515.5796508789062
    cx_d = 315.2550600044451
    cy_d = 245.1962086034237

    # #TODO_PR2_TOPIC_CHANGE
    # fx_d = 523.1627536752652
    # fy_d = 524.3198532120056
    # cx_d = 323.4825472948233
    # cy_d = 257.7431037652384

    # from: http://nicolas.burrus.name/index.php/Research/KinectCalibration
    x = (x_d - cx_d) * depth(x_d,y_d) / fx_d
    y = (y_d - cy_d) * depth(x_d,y_d) / fy_d
    z = depth(x_d,y_d)

    return (x,y,z)


# Handle of service
def handle_conv_coord(req):
    global listener

    # calculate the transform of the point in the raw image to
    # the grasping window in the depth point cloud!
    # TODO! Because this is hard!!!!! T.T

    x,y,z = convert2Dto3D(req.y,req.x)
    print x,y,z

    # convert from /camera_rgb_optical_frame to /camera_depth_optical_frame
    now = rospy.Time(0)
    # # TODO_PR2_TOPIC_CHANGE
    # # listener.waitForTransform('/camera_depth_optical_frame', '/camera_rgb_optical_frame', now, rospy.Duration(3.0))
    # listener.waitForTransform('/head_mount_kinect_ir_optical_frame', '/head_mount_kinect_rgb_optical_frame', now, rospy.Duration(3.0))
    # # TODO_PR2_TOPIC_CHANGE
    # # (trans, rot) = listener.lookupTransform( '/camera_depth_optical_frame', '/camera_rgb_optical_frame', now)
    # (trans, rot) = listener.lookupTransform( '/head_mount_kinect_ir_optical_frame', '/head_mount_kinect_rgb_optical_frame', now)

    # print trans,rot

    # mat = tf.TransformerROS()
    # rotMat = mat.fromTranslationRotation(trans,rot)

    # # print rotMat
    # pnt = np.asarray([x,y,z,1]).reshape(4,1)
    # # print pnt
    # # print np.matrix(rotMat)*pnt
    # newPnt = rotMat.dot(pnt)
    # newX = (newPnt[0]/newPnt[3])[0]
    # newY = (newPnt[1]/newPnt[3])[0]
    # newZ = (newPnt[2]/newPnt[3])[0]

    # pnt = PointStamped()
    # pnt.header.frame_id = "head_mount_kinect_rgb_optical_frame"
    # pnt.header.stamp = now
    # pnt.point.x = x
    # pnt.point.y = y
    # pnt.point.z = z

    # newPnt = listener.transformPoint("/head_mount_kinect_ir_optical_frame", pnt)

    # print newPnt

    # # print newX,newY,newZ
    # print "\n\n\n\n"
    # print "[{},{},{}]".format(newPnt.point.x,newPnt.point.y,newPnt.point.z)
    # print (newPnt.point.x,newPnt.point.y,newPnt.point.z)
    # return (newPnt.point.x,newPnt.point.y,newPnt.point.z)

    print "\n\n\n\n"
    print "[{},{},{}]".format(x,y,z)
    print (x,y,z)
    return (x,y,z)


# Server set up
def conv_coord_server():
    global listener 

    # Create a ROS node.
    rospy.init_node('conv_coord_server')

    listener = tf.TransformListener(True, rospy.Duration(10.0))

    # Create a service
    s = rospy.Service('conv_coord', Conv2DTo3D, handle_conv_coord)

    # spin service   
    print "Ready to convert coords from 2D to 3D!"
    rospy.spin()

#------------------------------------------------
if __name__ == "__main__":
    conv_coord_server()

