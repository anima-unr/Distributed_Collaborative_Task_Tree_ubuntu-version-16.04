#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters

# globals
pub_depth_reg_image_raw = rospy.Publisher('/local/depth_registered/image_raw', Image, queue_size=10)
pub_depth_reg_sw_image_rect_raw = rospy.Publisher('/local/depth_registered/sw_registered/image_rect_raw', Image, queue_size=10)
pub_depth_reg_hw_image_rect_raw = rospy.Publisher('/local/depth_registered/hw_registered/image_rect_raw', Image, queue_size=10)
pub_depth_reg_camera_info = rospy.Publisher('/local/depth_registered/camera_info', CameraInfo, queue_size=10)

pub_depth_camera_info = rospy.Publisher('/local/depth/camera_info', CameraInfo, queue_size=10)
pub_depth_image_raw = rospy.Publisher('/local/depth/image_raw', Image, queue_size=10)
pub_depth_image_rect_raw = rospy.Publisher('/local/depth/image_rect_raw', Image, queue_size=10)

pub_rgb_image_raw = rospy.Publisher('/local/rgb/image_raw', Image, queue_size=10)
pub_rgb_image_color = rospy.Publisher('/local/rgb/image_color', Image, queue_size=10)
pub_rgb_camera_info = rospy.Publisher('/local/rgb/camera_info', CameraInfo, queue_size=10)
pub_rgb_image_rect_color = rospy.Publisher('/local/rgb/image_rect_color', Image, queue_size=10)

reg_camera_info = []
depth_camera_info = []
rgb_camera_info = []




# # depth reg
# def callback_depth_reg_camera_info(data):
#     global pub_depth_reg_camera_info
#     pub_depth_reg_camera_info.publish(data)

# def callback_depth_reg_image_raw(image, info_sub):
#     global pub_depth_reg_image_raw
#     pub_depth_reg_image_raw.publish(image)

# def callback_depth_reg_sw_image_rect_raw(image, info_sub):
#     global pub_depth_reg_sw_image_rect_raw
#     pub_depth_reg_sw_image_rect_raw.publish(image)

# def callback_depth_reg_hw_image_rect_raw(image, info_sub):
#     global pub_depth_reg_hw_image_rect_raw
#     pub_depth_reg_hw_image_rect_raw.publish(image)


# # depth
# def callback_depth_camera_info(data):
#     global pub_depth_camera_info
#     pub_depth_camera_info.publish(data)

# def callback_depth_image_raw(image, info_sub):
#     global pub_depth_image_raw
#     pub_depth_image_raw.publish(image)

# def callback_depth_image_rect_raw(image, info_sub):
#     global pub_depth_image_rect_raw
#     pub_depth_image_rect_raw.publish(image)


# #rgb
# def callback_rgb_camera_info(data):
#     global pub_rgb_camera_info
#     pub_rgb_camera_info.publish(data)

# def callback_rgb_image_raw(image, info_sub):
#     global pub_rgb_image_raw
#     pub_rgb_image_raw.publish(image)

# def callback_rgb_image_color(image, info_sub):
#     global pub_rgb_image_color
#     pub_rgb_image_color.publish(image)


# def callback_rgb_image_rect_color(image, info_sub):
#     global pub_rgb_image_rect_color
#     pub_rgb_image_rect_color.publish(image)



# # main listener stuffs
# def listener():

#     # init
#     rospy.init_node('remapKinect2Local', anonymous=True)

#     # # subs
#     rospy.Subscriber("/kinect_head/depth_registered/camera_info", CameraInfo, callback_depth_reg_camera_info)
#     info_reg_sub = message_filters.Subscriber("/kinect_head/depth_registered/camera_info", CameraInfo)

#     image_reg_raw_sub = message_filters.Subscriber("/kinect_head/depth_registered/image_raw", Image)
#     ts1 = message_filters.TimeSynchronizer([image_reg_raw_sub, info_reg_sub], 10)
#     ts1.registerCallback(callback_depth_reg_image_raw)

#     image_reg_sw_rect_sub = message_filters.Subscriber("/kinect_head/depth_registered/sw_registered/image_rect_raw", Image)
#     ts2 = message_filters.TimeSynchronizer([image_reg_sw_rect_sub, info_reg_sub], 10)
#     ts2.registerCallback(callback_depth_reg_sw_image_rect_raw)

#     image_reg_hw_rect_sub = message_filters.Subscriber("/kinect_head/depth_registered/hw_registered/image_rect_raw", Image)
#     ts3 = message_filters.TimeSynchronizer([image_reg_hw_rect_sub, info_reg_sub], 10)
#     ts3.registerCallback(callback_depth_reg_hw_image_rect_raw)



#     rospy.Subscriber("/kinect_head/depth/camera_info", CameraInfo, callback_depth_camera_info)
#     info_depth_sub = message_filters.Subscriber("/kinect_head/depth/camera_info", CameraInfo)

#     image_depth_raw_sub = message_filters.Subscriber("/kinect_head/depth/image_raw", Image)
#     ts4 = message_filters.TimeSynchronizer([image_depth_raw_sub, info_depth_sub], 10)
#     ts4.registerCallback(callback_depth_image_raw)

#     image_depth_rect_sub = message_filters.Subscriber("/kinect_head/depth/image_rect_raw", Image)
#     ts5 = message_filters.TimeSynchronizer([image_depth_rect_sub, info_depth_sub], 10)
#     ts5.registerCallback(callback_depth_image_rect_raw)



#     rospy.Subscriber("/kinect_head/rgb/camera_info", CameraInfo, callback_rgb_camera_info)
#     info_rgb_sub = message_filters.Subscriber("/kinect_head/rgb/camera_info", CameraInfo)

#     image_rgb_raw_sub = message_filters.Subscriber("/kinect_head/rgb/image_raw", Image)
#     ts6 = message_filters.TimeSynchronizer([image_rgb_raw_sub, info_rgb_sub], 10)
#     ts6.registerCallback(callback_rgb_image_raw)

#     image_rgb_color_sub = message_filters.Subscriber("/kinect_head/rgb/image_color", Image)
#     ts7 = message_filters.TimeSynchronizer([image_rgb_color_sub, info_rgb_sub], 10)
#     ts7.registerCallback(callback_rgb_image_color)

#     image_rgb_rect_sub = message_filters.Subscriber("/kinect_head/rgb/image_rect_color", Image)
#     ts8 = message_filters.TimeSynchronizer([image_rgb_rect_sub, info_rgb_sub], 10)
#     ts8.registerCallback(callback_rgb_image_rect_color)

#     # spin
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         listener()
#     except rospy.ROSInterruptException:
#         pass




















##################################################################################################################################
##################################################################################################################################
##################################################################################################################################
##################################################################################################################################
##################################################################################################################################
##################################################################################################################################
##################################################################################################################################
##################################################################################################################################

















# # #!/usr/bin/env python
# # # license removed for brevity
# # import rospy
# # from std_msgs.msg import String
# # from sensor_msgs.msg import Image, CameraInfo
# # # import message_filters

# # globals
# pub_depth_reg_image_raw = rospy.Publisher('/local/depth_registered/image_raw', Image, queue_size=10)
# pub_depth_reg_sw_image_rect_raw = rospy.Publisher('/local/depth_registered/sw_registered/image_rect_raw', Image, queue_size=10)
# pub_depth_reg_hw_image_rect_raw = rospy.Publisher('/local/depth_registered/hw_registered/image_rect_raw', Image, queue_size=10)
# pub_depth_reg_camera_info = rospy.Publisher('/local/depth_registered/camera_info', CameraInfo, queue_size=10)

# pub_depth_camera_info = rospy.Publisher('/local/depth/camera_info', CameraInfo, queue_size=10)
# pub_depth_image_raw = rospy.Publisher('/local/depth/image_raw', Image, queue_size=10)
# pub_depth_image_rect_raw = rospy.Publisher('/local/depth/image_rect_raw', Image, queue_size=10)

# pub_rgb_image_raw = rospy.Publisher('/local/rgb/image_raw', Image, queue_size=10)
# pub_rgb_image_color = rospy.Publisher('/local/rgb/image_color', Image, queue_size=10)
# pub_rgb_camera_info = rospy.Publisher('/local/rgb/camera_info', CameraInfo, queue_size=10)
# pub_rgb_image_rect_color = rospy.Publisher('/local/rgb/image_rect_color', Image, queue_size=10)


# depth reg
def callback_depth_reg_camera_info(data):
    # global pub_depth_reg_camera_info
    # pub_depth_reg_camera_info.publish(data)
    global reg_camera_info
    reg_camera_info = data

def callback_depth_reg_image_raw(data):
    global pub_depth_reg_image_raw
    global pub_depth_reg_camera_info
    global reg_camera_info

    info = reg_camera_info
    info.header.stamp = data.header.stamp
    pub_depth_reg_camera_info.publish(info)
    pub_depth_reg_image_raw.publish(data)

def callback_depth_reg_sw_image_rect_raw(data):
    global pub_depth_reg_sw_image_rect_raw
    global pub_depth_reg_camera_info
    global reg_camera_info

    info = reg_camera_info
    info.header.stamp = data.header.stamp
    pub_depth_reg_camera_info.publish(info)
    pub_depth_reg_sw_image_rect_raw.publish(data)

def callback_depth_reg_hw_image_rect_raw(data):
    global pub_depth_reg_hw_image_rect_raw
    global pub_depth_reg_camera_info
    global reg_camera_info

    info = reg_camera_info
    info.header.stamp = data.header.stamp
    pub_depth_reg_camera_info.publish(info)
    pub_depth_reg_hw_image_rect_raw.publish(data)




# depth
def callback_depth_camera_info(data):
    # global pub_depth_camera_info
    # pub_depth_camera_info.publish(data)
    global depth_camera_info
    depth_camera_info = data

def callback_depth_image_raw(data):
    global pub_depth_image_raw
    global pub_depth_camera_info
    global depth_camera_info

    info = depth_camera_info
    info.header.stamp = data.header.stamp
    pub_depth_camera_info.publish(info)
    pub_depth_image_raw.publish(data)

def callback_depth_image_rect_raw(data):
    global pub_depth_image_rect_raw
    global pub_depth_camera_info
    global depth_camera_info

    info = depth_camera_info
    info.header.stamp = data.header.stamp
    pub_depth_camera_info.publish(info)
    pub_depth_image_rect_raw.publish(data)




#rgb
def callback_rgb_camera_info(data):
    # global pub_rgb_camera_info
    # pub_rgb_camera_info.publish(data)
    global rgb_camera_info
    rgb_camera_info = data

def callback_rgb_image_raw(data):
    global pub_rgb_image_raw
    global pub_rgb_camera_info
    global rgb_camera_info

    info = rgb_camera_info
    info.header.stamp = data.header.stamp
    pub_rgb_camera_info.publish(info)
    pub_rgb_image_raw.publish(data)

def callback_rgb_image_color(data):
    global pub_rgb_image_color
    global pub_rgb_camera_info
    global rgb_camera_info

    info = rgb_camera_info
    info.header.stamp = data.header.stamp
    pub_rgb_camera_info.publish(info)
    pub_rgb_image_color.publish(data)

def callback_rgb_image_rect_color(data):
    global pub_rgb_image_rect_color
    global pub_rgb_camera_info
    global rgb_camera_info

    info = rgb_camera_info
    info.header.stamp = data.header.stamp
    pub_rgb_camera_info.publish(info)
    pub_rgb_image_rect_color.publish(data)

# main listener stuffs
def listener():

    # init
    rospy.init_node('remapKinect2Local', anonymous=True)

    # for kinect on PR2: TODO_PR2_Topic_Change
    rospy.Subscriber("/kinect_head/depth_registered/camera_info", CameraInfo, callback_depth_reg_camera_info)
    rospy.Subscriber("/kinect_head/depth_registered/image_raw", Image, callback_depth_reg_image_raw)
    # rospy.Subscriber("/kinect_head/depth_registered/sw_registered/image_rect_raw", Image, callback_depth_reg_sw_image_rect_raw)
    # rospy.Subscriber("/kinect_head/depth_registered/hw_registered/image_rect_raw", Image, callback_depth_reg_hw_image_rect_raw)

    rospy.Subscriber("/kinect_head/depth/camera_info", CameraInfo, callback_depth_camera_info)
    rospy.Subscriber("/kinect_head/depth/image_raw", Image, callback_depth_image_raw)
    # rospy.Subscriber("/kinect_head/depth/image_rect_raw", Image, callback_depth_image_rect_raw)

    rospy.Subscriber("/kinect_head/rgb/camera_info", CameraInfo, callback_rgb_camera_info)
    rospy.Subscriber("/kinect_head/rgb/image_raw", Image, callback_rgb_image_raw)
    # rospy.Subscriber("/kinect_head/rgb/image_color", Image, callback_rgb_image_color)
    # rospy.Subscriber("/kinect_head/rgb/image_rect_color", Image, callback_rgb_image_rect_color)

    # # for kinect on local computer
    # rospy.Subscriber("/camera/depth_registered/camera_info", CameraInfo, callback_depth_reg_camera_info)
    # rospy.Subscriber("/camera/depth_registered/image_raw", Image, callback_depth_reg_image_raw)
    # rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw", Image, callback_depth_reg_sw_image_rect_raw)
    # rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect_raw", Image, callback_depth_reg_hw_image_rect_raw)

    # rospy.Subscriber("/camera/depth/camera_info", CameraInfo, callback_depth_camera_info)
    # rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth_image_raw)
    # rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback_depth_image_rect_raw)

    # rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, callback_rgb_camera_info)
    # rospy.Subscriber("/camera/rgb/image_raw", Image, callback_rgb_image_raw)
    # rospy.Subscriber("/camera/rgb/image_color", Image, callback_rgb_image_color)
    # rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_rgb_image_rect_color)



    # spin
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
