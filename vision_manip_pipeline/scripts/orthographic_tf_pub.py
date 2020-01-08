#!/usr/bin/env python
import rospy
import sys
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
  # if len(sys.argv) < 8 :
  #   rospy.logerr('Invalid number of parameters\nusage: ./static_turtle_tf2_broadcaster.py child_frame_name x y z roll pitch yaw')
  #   sys.exit(0)
  # else: 
  #   if sys.argv[1] == 'world':
  #     rospy.logerr('Your static turtle name cannot be "world"')
  #     sys.exit(0)
  
    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
  
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "/base"
    static_transformStamped.child_frame_id = "/test"
  
    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = -1.0
    static_transformStamped.transform.translation.z = 1.0
  
    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 1
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 0
 
    rate = rospy.Rate(20)  
    while not rospy.is_shutdown():
        print "Publishing!"
        broadcaster.sendTransform(static_transformStamped)
        rate.sleep()
