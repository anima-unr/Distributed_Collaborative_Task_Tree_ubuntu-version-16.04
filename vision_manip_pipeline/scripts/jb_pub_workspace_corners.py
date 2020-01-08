#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Code for project 1

import rospy
import sys
import getopt
import numpy as np
from std_msgs.msg import String
from visualization_msgs.msg import Marker 

def talker(data = []):
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    workspace = rospy.get_param("/detect_grasps/workspace")
    workspace_grasps = rospy.get_param("/detect_grasps/workspace_grasps")

    print workspace
    print workspace_grasps

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_workspace(pub, workspace, workspace_grasps)
        rate.sleep()

# ---------------------------------------------------------------------------------

def pub_workspace(pub, workspace, workspace_grasps):

    # get corners of workspace
    corners = getCornersFromParam(workspace)

    # for each corner in workspace, pub cube
    idx = 0
    for corner in corners:
        pub_cube(pub, corner, idx)
        idx += 1

    # get corners of workspace_grasps
    # corners_grasp = getCornersFromParam(workspace_grasps)

    # for each corner in workspace_grasps, pub cube
    # idx += 3
    # for corner in corners_grasp:
        # pub_cube(pub, corner, idx)
        # idx += 1

def getCornersFromParam(wkspace):

    # setup:
    # coord = min/max
    #     x = front/back
    #     y = right/left
    #     z = bottom/top
    front = wkspace[0]
    back = wkspace[1]
    right = wkspace[2]
    left = wkspace[3]
    bottom = wkspace[4]
    top = wkspace[5]
    corners = []

    # front right bottom
    corners.append([front,right,bottom])

    # front right top
    corners.append([front,right,top])

    # front left bottom
    corners.append([front,left,bottom])

    # front left top
    corners.append([front,left,top])

    # back right bottom
    corners.append([back,right,bottom])

    # back right top
    corners.append([back,right,top])

    # back left bottom
    corners.append([back,left,bottom])

    # back left top
    corners.append([back,left,top])

    print corners
    return corners


def pub_cube(pub, pos, idx):
    cube = Marker()
    # TODO_PR2_TOPIC_CHANGE
    # cube.header.frame_id = "/camera_depth_optical_frame"
    # cube.header.frame_id = "/head_mount_kinect_rgb_optical_frame"
    cube.header.frame_id = "/test"
    cube.header.stamp = rospy.Time.now()
    cube.type = cube.CUBE
    cube.action = cube.ADD
    cube.pose.position.x = pos[0]
    cube.pose.position.y = pos[1]
    cube.pose.position.z = pos[2]
    # cube.pose.orientation.w = -0.24316289018035953
    # cube.pose.orientation.x =  -0.07259004329567947
    # cube.pose.orientation.y = 0.1530028298082824
    # cube.pose.orientation.z =  0.6767718572399328
    cube.pose.orientation.w = 1
    cube.pose.orientation.x = 0
    cube.pose.orientation.y = 0
    cube.pose.orientation.z = 0


    cube.ns = 'cube'
    cube.scale.x = 0.05
    cube.scale.y = 0.05
    cube.scale.z = 0.05
    cube.color.b = 1.0
    cube.color.r = float(idx)/15
    cube.color.a = 0.8
    cube.id = idx
    rospy.loginfo('\nCube Pos:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.position.x, cube.pose.position.y, cube.pose.position.z))
    rospy.loginfo('\nCube Ori:\n\tx: {}\ty: {}\tz: {}'.format(cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z))
    pub.publish(cube)
    # return cube


# ---------------------------------------------------------------------------------

if __name__ == '__main__':
    talker()
