#!/usr/bin/env python

import collections

# from copy import deepcopy
import copy

import rospy

import tf
import cv2
import cv_bridge
import rospkg
import tf
import sys, argparse

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

from std_msgs.msg import Header

from sensor_msgs.msg import (
    Image,
    JointState,
)

import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from unr_manipulation.srv import *

import moveit_commander
import moveit_msgs.msg

import threading as thread
import pdb

# TODO: JB Added
import moveit_commander
import moveit_msgs.msg
import roslib

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
# Pick and place enum
STATE = enum('APPROACHING', 'PICKING', 'PICKED', 'PLACING', 'PLACED', 'NEUTRAL', 'IDLE')

class PickPlace(object):
    def __init__(self, limb, side):
        # TODO: Replace limb motion with moveit motions
        self.side = side
        # TODO: JB Added
        # self._limb = baxter_interface.Limb(limb)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self._limb = moveit_commander.MoveGroupCommander(limb)
        self._limb.set_max_velocity_scaling_factor(0.4);

        circle_io = baxter_interface.DigitalIO(side + '_lower_button')
        dash_io = baxter_interface.DigitalIO(side + '_upper_button')
        self.calibrating = False
        self.object_calib = -1
        self.objects = ['neutral', 'Cup', 'Tea', 'Sugar', 'Left_Bread', 'Right_Bread', 'Lettuce', 'Meat']
        # self.objects = ['neutral', 'placemat', 'cup', 'plate', 'fork', 'spoon', 'knife', 'bowl', 'soda', 'wineglass']
        # self.objects = ['neutral',  'cup', 'plate', 'bowl']
        # self.objects = ['neutral', 'bowl']
        # TODO: JB Added
        # self.object_pick_joint_angles = dict()
        self.object_pick_poses = dict()
        # TODO: JB Added
        # self.object_place_joint_angles = dict()
        self.object_place_poses = dict()


        self._gripper = baxter_interface.Gripper(side)
        self._gripper.calibrate()
        self._gripper.set_holding_force(100.0)

        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        circle_io.state_changed.connect(self.CirclePressed_)

        # Variables to manage threads
        self.work_thread = None
        self.stop = False

        # state variables
        self.state = STATE.IDLE

    def CirclePressed_(self, value):
        if value and self.object_calib != -1:
            if self.pick:
                print "Calibrating Object Pick: " + self.objects[self.object_calib] + "..."
                # TODO: JB Added
                # self.object_pick_joint_angles[self.objects[self.object_calib]] = self._limb.joint_angles()
                # self.object_pick_poses[self.objects[self.object_calib]] = self._limb.endpoint_pose()
                pose = self.GetArmPoseGoal()
                self.object_pick_poses[self.objects[self.object_calib]] = pose
                print(pose)
                print "Calibrated"
                self.calibrating = False
            else:
                # TODO: JB Added
                # self.object_place_joint_angles[self.objects[self.object_calib]] = self._limb.joint_angles()
                # self.object_place_poses[self.objects[self.object_calib]] = self._limb.endpoint_pose()
                pose = self.GetArmPoseGoal()
                self.object_place_poses[self.objects[self.object_calib]] = pose
                print(pose)
                print "Calibrated"
                self.calibrating = False


    # TODO: JB Added
    def GetArmPoseGoal(self):

        # set up the things
        pose_target = Pose()
        now = rospy.Time(0)

        # get the tf
        t = tf.TransformListener(True, rospy.Duration(10.0))
        t.waitForTransform("/base", "/right_gripper", now,rospy.Duration(3.0));
        pose = t.lookupTransform("/base", "/right_gripper", now)

        # format pose to correct format
        pose_target.orientation.x = pose[1][0]
        pose_target.orientation.y = pose[1][1]
        pose_target.orientation.z = pose[1][2]
        pose_target.orientation.w = pose[1][3]
        pose_target.position.x = pose[0][0] 
        pose_target.position.y = pose[0][1] 
        pose_target.position.z = pose[0][2]

        self._limb.set_pose_target(pose_target)
        self._limb.set_num_planning_attempts(3);
        self._limb.set_planning_time(5.0);
        self._limb.set_goal_position_tolerance(0.0075)
        self._limb.set_goal_orientation_tolerance(0.0075)

        return pose_target


    def moveToPose(self, pose):

        # define temp pose
        pose_target = Pose()

        # format the pose correctly
        print pose
        pose_target.orientation.x = pose.orientation.x
        pose_target.orientation.y = pose.orientation.y
        pose_target.orientation.z = pose.orientation.z
        pose_target.orientation.w = pose.orientation.w
        pose_target.position.x = pose.position.x
        pose_target.position.y = pose.position.y
        pose_target.position.z = pose.position.z

        # set things
        self._limb.set_pose_target(pose_target)
        self._limb.set_num_planning_attempts(5);
        self._limb.set_planning_time(10.0);
        self._limb.set_goal_position_tolerance(0.0075)
        self._limb.set_goal_orientation_tolerance(0.0075)

        print("\tPlanning...")
        plan1 = self._limb.plan()
        # rospy.sleep(5)
        print("\tExecuting...")
        self._limb.go(wait=True)
        # rospy.sleep(5)


    def PickAndPlaceImpl(self, req):

        if self.stop:
            return
        print "Picking UP Object: " + req.object
        # self._limb.set_joint_position_speed(0.2)
        if self.stop:
            return
        self.state = STATE.NEUTRAL
        # TODO: JB Added
        # self._limb.move_to_joint_positions(self.object_pick_joint_angles['neutral'])
        # self.moveToPose(self.object_pick_poses['neutral'])
        pick_pose_offset = copy.deepcopy(self.object_pick_poses)
        pick_pose_offset = pick_pose_offset[req.object]
        pick_pose_offset.position.z = pick_pose_offset.position.z + 0.2; 
        # pick_pose_offset.position.x = pick_pose_offset.position.x - 0.1; 
        # pick_pose_offset.position.y = pick_pose_offset.position.y - 0.1; 
        rospy.sleep(3.0)
        self.moveToPose(pick_pose_offset)
        if self.stop:
            return
        self._gripper.command_position(100.0)
        self.state = STATE.APPROACHING
        if self.stop:
            return
        rospy.sleep(3.0)
        if self.stop:
            return
        self.state = STATE.PICKING
        # self._limb.move_to_joint_positions(self.object_pick_joint_angles[req.object])
        self.moveToPose(self.object_pick_poses[req.object])
        if self.stop:
            return
        self._gripper.command_position(0.0)
        self.state = STATE.PICKED
        if self.stop:
            return
        rospy.sleep(3.0)
        if self.stop:
            return
        # self._limb.move_to_joint_positions(self.object_pick_joint_angles['neutral'])
        # self.moveToPose(self.object_pick_poses['neutral'])
        pick_pose_offset = copy.deepcopy(self.object_pick_poses)
        pick_pose_offset = pick_pose_offset[req.object]
        pick_pose_offset.position.z = pick_pose_offset.position.z + 0.2; 
        # pick_pose_offset.position.x = pick_pose_offset.position.x - 0.1; 
        # pick_pose_offset.position.y = pick_pose_offset.position.y - 0.1; 
        self.moveToPose(pick_pose_offset)
        if self.stop:
            return
        print "Placing Down Object:" + req.object
        self.state = STATE.PLACING
        rospy.sleep(3.0)
        if self.stop:
            return
        # self._limb.move_to_joint_positions(self.object_place_joint_angles['neutral'])
        # self.moveToPose(self.object_place_poses['neutral'])
        place_pose_offset = copy.deepcopy(self.object_place_poses)
        place_pose_offset = place_pose_offset[req.object]
        place_pose_offset.position.z = place_pose_offset.position.z + 0.2; 
        # place_pose_offset.position.x = place_pose_offset.position.x - 0.1; 
        # place_pose_offset.position.y = place_pose_offset.position.y - 0.1; 
        self.moveToPose(place_pose_offset)
        rospy.sleep(3.0)
        if self.stop:
            return
        # self._limb.move_to_joint_positions(self.object_place_joint_angles[req.object])
        self.moveToPose(self.object_place_poses[req.object])
        if self.stop:
            return
        self._gripper.command_position(100.0)
        self.state = STATE.PLACED
        if self.stop:
            return
        rospy.sleep(3.0)
        if self.stop:
            return
        # self._limb.move_to_joint_positions(self.object_place_joint_angles['neutral'])
        # self.moveToPose(self.object_place_poses['neutral'])
        place_pose_offset = copy.deepcopy(self.object_place_poses)
        place_pose_offset = place_pose_offset[req.object]
        place_pose_offset.position.z = place_pose_offset.position.z + 0.2; 
        # place_pose_offset.position.x = place_pose_offset.position.x - 0.1; 
        # place_pose_offset.position.y = place_pose_offset.position.y - 0.1; 
        self.moveToPose(place_pose_offset)
        if self.stop:
            return
        rospy.sleep(3.0)
        self._gripper.command_position(0.0)
        if self.stop:
            return
        self.state = STATE.IDLE

    def PickAndPlaceSendGoal(self):
        display_trajectory_publisher = rospy.Publisher(
                            '/move_group/display_planned_path',
                            moveit_msgs.msg.DisplayTrajectory)
        # THE WAY LUKE DID IT:
        # self._limb.move_to_joint_positions(self.object_pick_joint_angles['neutral'])

        # TODO: Replace limb motion with moveit motions
        # pose_target = geometry_msgs.msg.Pose()
        # pose_target.orientation.w = 1.0
        # pose_target.position.x = 0.7
        # pose_target.position.y = -0.05
        # pose_target.position.z = 1.1
        # group.set_pose_target(pose_target)
        # plan1 = group.plan()

    def PickAndPlaceObject(self, req):
        # starting a thread that will handle the pick and place.
        self.stop = True
        if self.work_thread != None and self.work_thread.is_alive():
                self.work_thread.join()
        self.stop = False
        self.work_thread = thread.Thread(target=self.PickAndPlaceImpl, args=[req])
        self.work_thread.start()
        return pick_and_placeResponse(True)

    def PickAndPlaceCheck(self, req):
        # checks to see if the pick and place is in the final placed state
        check = self.state == STATE.PLACED
        if check:
            self.state = STATE.IDLE
        return pick_and_placeResponse(check)

    def PickAndPlaceState(self, req):
        # return the state of the pick and place
        return pick_and_place_stateResponse(self.state)
    def PickAndPlaceStop(self, req):
        # Stop the arm from picking and placing
        if self.work_thread == None:
            return pick_and_place_stopResponse(False)
        self.stop = True
        self.work_thread.join()
        self.stop = False
        return pick_and_place_stopResponse(True)

    def CalibrateObjects(self):
        '''
        Calibrate Objects: cup
        '''
        print "Calibrating Objects:"
        # Calibrate Cup
        for i, object in enumerate(self.objects):
            print "Move " + self.side + " limb to: " + object + " picking location and Press Circle"
            self.pick = True
            self.object_calib = i
            self.calibrating = True
            while self.calibrating and not rospy.is_shutdown():
                rospy.sleep(.1)
            self._gripper.command_position(0.0)
            print "Move " + self.side + " limb to: " + object + " Placing location and Press Circle"
            self.pick = False
            self.calibrating = True
            while self.calibrating and not rospy.is_shutdown():
                rospy.sleep(.1)
            self._gripper.command_position(100.0)

        self.object_calib = -1

    def _find_approach(self, pose, offset):
        ikreq = SolvePositionIKRequest()
        # Add 5 cm offset in Z direction
        try:
            pose['position'] = Point(x=pose['position'][0],
                                     y=pose['position'][1],
                                     z=pose['position'][2] + offset
                                     )
        except Exception:
            pose['position'] = Point(x=pose['position'].x,
                                     y=pose['position'].y,
                                     z=pose['position'].z + offset
                                     )
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _find_jp(self, pose):
        ikreq = SolvePositionIKRequest()

        goal_pose = Pose()
        goal_pose.position = pose['position']
        goal_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=goal_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    # TODO: JB Added
    # def ReadCalibration(self, filename):
    #     with open(filename, 'r') as f:
    #         for line in f:
    #             split = line.split('%')
    #             location = split[0]
    #             pick_or_place = location.split('_')[0]
    #             key = location.split('_')[1]
    #             position = split[1]
    #             if pick_or_place == 'pickAngles':
    #                 self.object_pick_joint_angles[key] = eval(position)
    #             elif pick_or_place == 'pickPose':
    #                 self.object_pick_poses[key] = eval(position)
    #             elif pick_or_place == 'placeAngles':
    #                 self.object_place_joint_angles[key] = eval(position)
    #             elif pick_or_place == 'placePose':
    #                 self.object_place_poses[key] = eval(position)
    def ReadCalibration(self, filename):
        with open(filename, 'r') as f:
            # read base frame id and write end-effector frame id? -- not used so ignore!?!
            line = f.readline()
            base_frame_id = line.rstrip()
            line = f.readline()
            ee_frame_id = line.rstrip()
            print(base_frame_id)
            print(ee_frame_id)

            # until end of file
            while True:
                line = f.readline()
                if not line: 
                    break

                # read key
                key = line.rstrip()

                # read into pick pos at key
                line = f.readline()
                line = line.rstrip()                
                data = line.split(',')
                self.object_pick_poses[key] = Pose()
                self.object_pick_poses[key].position.x = float(data[0])
                self.object_pick_poses[key].position.y = float(data[1])
                self.object_pick_poses[key].position.z = float(data[2])

                # read into pick ori at key
                line = f.readline()
                line = line.rstrip()
                data = line.split(',')
                self.object_pick_poses[key].orientation.x = float(data[0])
                self.object_pick_poses[key].orientation.y = float(data[1])
                self.object_pick_poses[key].orientation.z = float(data[2])
                self.object_pick_poses[key].orientation.w = float(data[3])

                # read into place pos at key
                line = f.readline()
                line = line.rstrip()
                data = line.split(',')
                self.object_place_poses[key] = Pose()
                self.object_place_poses[key].position.x = float(data[0])
                self.object_place_poses[key].position.y = float(data[1])
                self.object_place_poses[key].position.z = float(data[2])

                # read into place ori at key
                line = f.readline()
                line = line.rstrip()
                data = line.split(',')
                self.object_place_poses[key].orientation.x = float(data[0])
                self.object_place_poses[key].orientation.y = float(data[1])
                self.object_place_poses[key].orientation.z = float(data[2])
                self.object_place_poses[key].orientation.w = float(data[3])

                print key
                print self.object_pick_poses[key]
                print self.object_place_poses[key]

        f.close()


    # TODO: JB Added
    # def SaveCalibration(self, filename):
    #     f = open(filename, 'w')
    #     for key in self.object_pick_joint_angles:
    #         f.write('pickAngles_' + key + '%' + str(self.object_pick_joint_angles[key]) + '\n')
    #         f.write('pickPose_' + key + '%' + str(self.object_pick_poses[key]) + '\n')
    #     for key in self.object_place_joint_angles:
    #         f.write('placeAngles_' + key + '%' + str(self.object_place_joint_angles[key]) + '\n')
    #         f.write('placePose_' + key + '%' + str(self.object_place_poses[key]) + '\n')
    #     f.close()
    def SaveCalibration(self, filename):
        f = open(filename, 'w')
        # write base frame id and write end-effector frame id?
        f.write('base\n' + self.side + '_gripper' + '\n')

        for key in self.object_pick_poses:
            # write object name
            f.write(key + '\n')
            # write pick pos
            f.write(str(self.object_pick_poses[key].position.x) + ',' +
                str(self.object_pick_poses[key].position.y) + ',' +
                str(self.object_pick_poses[key].position.z) +'\n')
            # write pick ori
            f.write(str(self.object_pick_poses[key].orientation.x) + ',' +
                str(self.object_pick_poses[key].orientation.y) + ',' +
                str(self.object_pick_poses[key].orientation.z) + ',' + 
                str(self.object_pick_poses[key].orientation.w) + '\n')
            # write place pos
            f.write(str(self.object_place_poses[key].position.x) + ',' +
                str(self.object_place_poses[key].position.y) + ',' +
                str(self.object_place_poses[key].position.z) +'\n')
            # write place ori
            f.write(str(self.object_place_poses[key].orientation.x) + ',' +
                str(self.object_place_poses[key].orientation.y) + ',' +
                str(self.object_place_poses[key].orientation.z) + ',' + 
                str(self.object_place_poses[key].orientation.w) + '\n')
        f.close()

    # TODO: JB Added
    def PostParameters(self):
        for key in self.object_pick_poses:
            # Post param for Pick Position
            rospy.set_param('/ObjectPositions/' + key,
                [self.object_pick_poses[key].position.x,
                 self.object_pick_poses[key].position.y,
                 self.object_pick_poses[key].position.z])


def main():
    rospy.init_node("pick_and_place_service")

    parser = argparse.ArgumentParser(description='Process Pick and Place Command Line Arguments')
    parser.add_argument('--save', '-s', type=str)
    parser.add_argument('--read', '-r', type=str)
    args = parser.parse_args()

    rs = baxter_interface.RobotEnable()
    print("Enabling Robot")
    rs.enable()
    pp = PickPlace('right_arm', 'right')

    # Calibrate Pickup Locations
    if args.read:
        print "Reading Position File: " + args.read
        pp.ReadCalibration(args.read)
    else:
        pp.CalibrateObjects()

    # Post Params
    pp.PostParameters();
    # Save Calibration to File
    if args.save:
        print "Saving Calibration File: " + args.save
        pp.SaveCalibration(args.save)

    # Advertise Service
    s = rospy.Service('pick_and_place_object', pick_and_place, pp.PickAndPlaceObject)

    s_2 = rospy.Service('pick_and_place_check', pick_and_place, pp.PickAndPlaceCheck)

    s_3 = rospy.Service('pick_and_place_state', pick_and_place_state, pp.PickAndPlaceState)

    s_4 = rospy.Service('pick_and_place_stop', pick_and_place_stop, pp.PickAndPlaceStop)

    print "READY to PICK and Place"
    rospy.spin()

if __name__ == '__main__':
    main()