#!/usr/bin/env python

import rospy
import tf
import numpy
from gpd.msg import GraspConfigList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *
from copy import deepcopy
import math

class GraspApproach:
  def __init__(self):
    rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.GraspCallback)
    self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
    self.pospub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
    self.index = 0

  def GraspCallback(self, grasps):
    if len(grasps.grasps) > 0:
      self.PubGraspApproach(grasps)

  def GraspToQuat(self, grasp):
    result = Quaternion(0,0,0,1)
    #R = ( (grasp.axis.x, grasp.binormal.x, grasp.approach.x), (grasp.axis.y, grasp.binormal.y, grasp.approach.y), (grasp.axis.z, grasp.binormal.z, grasp.approach.z) )
    #print R
    
    up = (grasp.binormal.x, grasp.binormal.y, grasp.binormal.z)
    forward = (grasp.axis.x, grasp.axis.y, grasp.axis.z)
    m2 = deepcopy(forward)
    m0 = numpy.cross(up, forward)
    m1 = numpy.cross(m2, m0)

    #print m0, m1, m2

    r0 = m0[0] + m1[1] + m2[2];
    if r0 > 0:
        r1 = math.sqrt(r0 + 1.0);
        result.w = r1 * 0.5;
        r1 = 0.5 / r1;
        result.x = (m1[2] - m2[1]) * r1
        result.y = (m2[0] - m0[2]) * r1
        result.z = (m0[1] - m1[0]) * r1
        return result
    elif ( m0[0] >= m1[1]) and (m0[0] >= m2[2]):
        r1 = math.sqrt(((1 + m0[0]) - m1[1]) - m2[2])
        r2 = 0.5 / r1
        result.x = 0.5 * r1
        result.y = (m0[1] + m1[0]) * r2
        result.z = (m0[2] + m2[0]) * r2
        result.w = (m1[2] - m2[1]) * r2
        return result
    elif (m1[1] > m2[2]):
        r1 = math.sqrt(((1 + m1[1]) - m0[0]) - m2[2])
        r2 = 0.5 / r1
        result.x = (m1[0] + m0[1]) * r2
        result.y = 0.5 * r1
        result.z = (m2[1] + m1[2]) * r2
        result.w = (m2[0] - m0[2]) * r2
        return result
    else:
        r1 = math.sqrt(((1 + m2[2]) - m0[0]) - m1[1])
        r2 = 0.5 / r1
        result.x = (m2[0] + m0[2]) * r2
        result.y = (m2[1] + m1[2]) * r2
        result.z = 0.5 * r1
        result.w = (m0[1] - m1[0]) * r2
        return result
    return result

  def PubGraspFrame(self, grasp):
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.id = 1
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 0.75
    marker.points = [grasp.bottom, self.addVector(grasp.bottom, grasp.axis)]
    self.pub.publish(marker)
    marker.color.r = 0
    marker.color.g = 1
    marker.id = 2
    marker.points = [grasp.bottom, self.addVector(grasp.bottom, grasp.binormal)]
    self.pub.publish(marker)
    marker.color.b = 1
    marker.color.g = 0
    marker.id = 3
    marker.points = [grasp.bottom, self.addVector(grasp.bottom, grasp.approach)]
    self.pub.publish(marker)


  def addVector(self, point1, point2):
    ret = Point()
    ret.x = point1.x + point2.x
    ret.y = point1.y + point2.y
    ret.z = point1.z + point2.z
    return ret

  def PubGraspApproach(self, grasps):
    base = deepcopy(grasps.grasps[0].bottom)
    vec = deepcopy(grasps.grasps[0].approach)
    marker = Marker()
    marker.header.frame_id = "camera_rgb_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = base
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05
    marker.scale.y = .05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    self.pub.publish(marker)
    marker.id = 1
    approach = base
    approach.x -= 0.08*vec.x
    approach.y -= 0.08*vec.y
    approach.z -= 0.08*vec.z
    marker.pose.position = deepcopy(approach)
    marker.color.r = 1
    marker.color.g = 0
    self.pub.publish(marker)

    self.PubGraspFrame(grasps.grasps[0])

    pose = PoseStamped()
    pose.header.frame_id = "camera_rgb_optical_frame"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = deepcopy(approach)
    # vec = grasps.grasps[0].axis
    # yaw = math.atan2(vec.y, vec.x);
    # pitch = math.atan2(math.sqrt((vec.x * vec.x) + (vec.y * vec.y)), vec.z );
    # roll = 0
    # print yaw, pitch, roll
    # pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw))
    pose.pose.orientation = self.GraspToQuat(grasps.grasps[0])
    self.pospub.publish(pose)

def init():
 rospy.init_node('grasp_approach')

 p = GraspApproach()
 rospy.spin()

if __name__ == '__main__':
 init()