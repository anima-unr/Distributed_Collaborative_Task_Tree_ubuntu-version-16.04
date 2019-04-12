#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from pcl_msgs.msg import ModelCoefficients
from visualization_msgs.msg import Marker

class PublishPlane:
  def __init__(self):
    rospy.Subscriber("/planar_segmentation/model", ModelCoefficients, self.ReceiveModel)
    self.pub = rospy.Publisher("/plane_display", Marker, queue_size=1)
    self.index = 0

  def ReceiveModel(self, model):
    self.PubDisplayPlaneDisplayMessage(model.values)

  def PubDisplayPlaneDisplayMessage(self, model):

    a = model[0]
    b = model[1]
    c = model[2]
    d = model[3]
    x_0 = 0
    y_0 = 0
    z_0 = (-a*x_0 - b*y_0 - d) / c

    marker = Marker()
    marker.header.frame_id = "camera_link"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x_0
    marker.pose.position.y = y_0
    marker.pose.position.z = z_0
    marker.pose.orientation.x = a
    marker.pose.orientation.y = b
    marker.pose.orientation.z = c
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = .1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = .5
    marker.color.g = .5
    marker.color.b = .1
    self.pub.publish(marker)
    self.old_marker = marker


    self.index += 1


def init():
  rospy.init_node('plane_Publish')

  p = PublishPlane()
  rospy.spin()

if __name__ == '__main__':
  init()