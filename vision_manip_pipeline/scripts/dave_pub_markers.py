#!/usr/bin/env python

import rospy
import copy
import numpy as np
from gpd.msg import GraspConfigList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


#==============================================================

def rotationQuat(axis, binormal, approach):

    rotation = {'w':0,'x':0,'y':0,'z':0}

    if approach.z < 0:
        if axis.x > binormal.y:
            trace = 1 + axis.x - binormal.y - approach.z
            rotation['w'] = trace
            rotation['x'] = axis.y + binormal.x
            rotation['y'] = approach.x + axis.z
            rotation['z'] = binormal.z - approach.y
        else:
            trace = 1 - axis.x + binormal.y - approach.z
            rotation['w'] = axis.y + binormal.x
            rotation['x'] = trace
            rotation['y'] = binormal.z + approach.y
            rotation['z'] = approach.x - axis.z
    else:
        if axis.x < (-binormal.y):
            trace = 1 - axis.x - binormal.y + approach.z
            rotation['w'] = approach.x + axis.z
            rotation['x'] = binormal.z + approach.y
            rotation['y'] = trace
            rotation['z'] = axis.y - binormal.x
        else:
            trace = 1 + axis.x + binormal.y + approach.z
            rotation['w'] = binormal.z - approach.y
            rotation['x'] = approach.x - axis.z
            rotation['y'] = axis.y - binormal.x
            rotation['z'] = trace

    rotation['w'] *= 0.5 / (trace **(0.5))
    rotation['x'] *= 0.5 / (trace **(0.5))
    rotation['y'] *= 0.5 / (trace **(0.5))
    rotation['z'] *= 0.5 / (trace **(0.5))

    return rotation


#==============================================================


class GraspApproach:
 def __init__(self):
   rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.GraspCallback)
   self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
   self.index = 0  

 def GraspCallback(self, grasps):
       if len(grasps.grasps) > 0:
               self.PubGraspApproach(grasps)

 def PubGraspApproach(self, grasps):
   base = copy.deepcopy(grasps.grasps[0].bottom)
   vec = copy.deepcopy(grasps.grasps[0].approach)
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
   marker.scale.x = 0.01
   marker.scale.y = 0.05
   marker.scale.z = 0.05
   marker.color.a = 1.0
   marker.color.r = 0
   marker.color.g = 1
   marker.color.b = 0
   self.pub.publish(marker)
   marker.id = 1
   approach = copy.deepcopy(base)
   approach.x -= 0.08*vec.x
   approach.y -= 0.08*vec.y
   approach.z -= 0.08*vec.z
   marker.pose.position = approach
   marker.color.r = 1
   marker.color.g = 0
   self.pub.publish(marker)


   # marker.pose.position = grasps.grasps[0].top
   # marker.type = Marker.CUBE
   # marker.id = 2
   marker.scale.x = 0.1
   marker.scale.y = 0.1
   marker.scale.z = 0.1
   marker.color.r = 0
   marker.color.g = 0
   marker.color.b = 1
   marker.color.a = 0.5
   # self.pub.publish(marker)
   # marker.pose.position = grasps.grasps[0].bottom
   # marker.type = Marker.CYLINDER
   # marker.id = 3
   # marker.color.r = 0
   # marker.color.g = 0.1
   # marker.color.b = 0.9
   # self.pub.publish(marker)
   # marker.pose.position = grasps.grasps[0].surface
   # marker.type = Marker.ARROW
   # marker.id = 4
   # marker.color.r = 0
   # marker.color.g = 0.2
   # marker.color.b = 0.8
   # self.pub.publish(marker)

   center = Point()
   center.x = base.x;   center.y = base.y;   center.z = base.z;

   # marker.pose.position = grasps.grasps[0].approach
   # marker.type = Marker.SPHERE
   # marker.id = 5
   # marker.color.r = 0
   # marker.color.g = 0.3
   # marker.color.b = 0.7
   # self.pub.publish(marker)
   # marker.pose.position = grasps.grasps[0].binormal
   # marker.type = Marker.CYLINDER
   # marker.id = 6
   # marker.color.r = 0.9
   # marker.color.g = 0.1
   # marker.color.b = 0.0
   # self.pub.publish(marker)
   # # marker.pose.position = grasps.grasps[0].axis
   marker.pose.position = copy.deepcopy(base)
   marker.type = Marker.SPHERE
   marker.action = Marker.ADD
   marker.id = 8
   marker.color.r = 0.5
   marker.color.g = 0.3
   marker.color.b = 0.7
   self.pub.publish(marker)
   # marker.pose.position = None
   marker2 = Marker()
   marker2.header.frame_id = "camera_rgb_optical_frame"
   marker2.header.stamp = rospy.Time.now()
   marker2.type = Marker.ARROW
   marker2.action = Marker.ADD
   test = Point()
   marker2.points = [base, grasps.grasps[0].axis]
   # marker.points
   marker2.id = 9
   marker2.scale.x = 0.05
   marker2.scale.y = 0.1
   marker2.scale.z = 0.0
   marker2.color.r = 0.8
   marker2.color.g = 0.2
   marker2.color.b = 0.0
   marker2.color.a = 0.5
   self.pub.publish(marker2)
   marker2.type = Marker.ARROW
   marker2.action = Marker.ADD
   # marker2.points[0] = approach
   # marker2.points[1] = grasps.grasps[0].binormal
   marker2.points = [base, grasps.grasps[0].binormal]
   marker2.id = 10
   marker2.scale.x = 0.05
   marker2.scale.y = 0.1
   marker2.scale.z = 0.0
   marker2.color.r = 0.5
   marker2.color.g = 0.5
   marker2.color.b = 0.0
   marker2.color.a = 0.5
   self.pub.publish(marker2)
   marker2.type = Marker.ARROW
   marker2.action = Marker.ADD
   # marker2.points[0] = approach
   # marker2.points[1] = grasps.grasps[0].approach
   marker2.points = [base, grasps.grasps[0].approach]
   marker2.id = 11
   marker2.scale.x = 0.05
   marker2.scale.y = 0.1
   marker2.scale.z = 0.0
   marker2.color.r = 0.0
   marker2.color.g = 0.5
   marker2.color.b = 0.5
   marker2.color.a = 0.5
   self.pub.publish(marker2)
   # marker2.type = Marker.ARROW
   # marker2.action = Marker.ADD
   # marker2.points[0] = approach
   # marker2.points[1] = grasps.grasps[0].bottom
   # marker2.id = 12
   # marker2.scale.x = 0.05
   # marker2.scale.y = 0.1
   # marker2.scale.z = 0.0
   # marker2.color.r = 0.5
   # marker2.color.g = 0.5
   # marker2.color.b = 0.5
   # marker2.color.a = 0.5
   # self.pub.publish(marker2)
   # marker2.type = Marker.ARROW
   # marker2.action = Marker.ADD
   # marker2.points[0] = approach
   # marker2.points[1] = grasps.grasps[0].top
   # marker2.id = 13
   # marker2.scale.x = 0.05
   # marker2.scale.y = 0.1
   # marker2.scale.z = 0.0
   # marker2.color.r = 0.5
   # marker2.color.g = 0.9
   # marker2.color.b = 0.5
   # marker2.color.a = 0.5
   # self.pub.publish(marker2)

   # center = Point()
   # center.x = 0;   center.y = 0;   center.z = 0;

   # marker2 = Marker()
   # marker2.pose.position = grasps.grasps[0].approach
   # marker2.header.frame_id = "camera_rgb_optical_frame"
   # marker2.header.stamp = rospy.Time.now()
   # marker2.type = Marker.ARROW
   # marker2.id = 5
   # # marker2.points.append(center)
   # # marker2.points.append(grasps.grasps[0].approach)
   # marker2.scale.x = 1
   # marker2.scale.y = 0.5
   # marker2.scale.z = 0.1
   # marker2.color.r = 0
   # marker2.color.g = 0.3
   # marker2.color.b = 0.7
   # self.pub.publish(marker2)
   # marker2.pose.position = grasps.grasps[0].binormal
   # # marker2.points[1] = grasps.grasps[0].binormal
   # marker2.type = Marker.ARROW
   # marker2.id = 6
   # marker2.color.r = 0.9
   # marker2.color.g = 0.1
   # marker2.color.b = 0.0
   # self.pub.publish(marker2)
   # marker2.pose.position = grasps.grasps[0].axis
   # # marker2.points[1] = grasps.grasps[0].axis
   # marker2.type = Marker.ARROW
   # marker2.id = 7
   # marker2.color.r = 0.8
   # marker2.color.g = 0.2
   # marker2.color.b = 0.0
   # self.pub.publish(marker2)
   # # marker.pose.position.x = 0;    marker.pose.position.y = 0;    marker.pose.position.z = 0;
   # # marker2.points[1] = grasps.grasps[0].binormal
   # # marker2.type = Marker.SPHERE
   # # marker2.id = 8
   # # marker2.color.r = 0.5
   # # marker2.color.g = 0.3
   # # marker2.color.b = 0.7
   # # self.pub.publish(marker)


   self.pubOrientation(grasps, base, vec, approach)


 def pubOrientation(self, grasps, base, vec, approach):
   
   # calc the orientation using mariya's code

   print grasps.grasps[0]

   axis_g = copy.deepcopy(grasps.grasps[0].axis)
   binormal_g = copy.deepcopy(grasps.grasps[0].binormal)
   approach_g = copy.deepcopy(grasps.grasps[0].approach)
   # approach_g.x = -approach_g.x;    approach_g.y = -approach_g.y;    approach_g.z = -approach_g.z; 
   # binormal_g.x = -binormal_g.x;    binormal_g.y = -binormal_g.y;    binormal_g.z = -binormal_g.z; 
   # axis_g.x = -axis_g.x;    axis_g.y = -axis_g.y;    axis_g.z = -axis_g.z; 
   rot = rotationQuat(axis_g, binormal_g, approach_g)


   #  fix rotation.....
   # t = tf.Transformer(True, rospy.Duration(10.0))
   # t->asMatrx("camera_rgb_optical_frame", "")

   # publish an arrow from the approach at the orientation
   #   -> arrow should point to the green dot if orientation is correct 
   marker = Marker()
   marker.header.frame_id = "camera_rgb_optical_frame"
   marker.header.stamp = rospy.Time.now()
   marker.id = 20
   marker.type = Marker.ARROW
   marker.action = Marker.ADD
   marker.pose.position = approach
   marker.pose.orientation.x = rot['x']
   marker.pose.orientation.y = rot['y']
   marker.pose.orientation.z = rot['z']
   marker.pose.orientation.w = rot['w']
   # marker.points[0] = approach
   # marker.points[1] = base
   marker.scale.x = 0.05
   marker.scale.y = 0.02
   marker.scale.z = 0.01
   marker.color.a = 1.0
   marker.color.r = 0.5
   marker.color.g = 0
   marker.color.b = 0.5
   self.pub.publish(marker)

   # get vector between points
   newVec = copy.deepcopy(approach);
   newVec.x -= base.x;
   newVec.y -= base.y;
   newVec.z -= base.z;

   # mornalize to unit 1
   normed = copy.deepcopy(newVec)
   mag = newVec.x*newVec.x + newVec.y*newVec.y + newVec.z*newVec.z
   normed.x = normed.x / np.sqrt(newVec.x*newVec.x + newVec.y*newVec.y + newVec.z*newVec.z)
   normed.y = normed.y / np.sqrt(newVec.x*newVec.x + newVec.y*newVec.y + newVec.z*newVec.z)
   normed.z = normed.z / np.sqrt(newVec.x*newVec.x + newVec.y*newVec.y + newVec.z*newVec.z)

   print newVec
   print mag
   print normed

   # marker.pose.position = approach
   # marker.pose.orientation.x = rot['x']
   # marker.pose.orientation.y = rot['y']
   # marker.pose.orientation.z = rot['z']
   # marker.pose.orientation.w = rot['w']
   # self.pub.publish(marker)


def init():
 rospy.init_node('grasp_approach')
 p = GraspApproach()
 rospy.spin()

if __name__ == '__main__':
 init()