#ifndef PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE
#define PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE

#include "table_setting_demo/pick_and_place.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

//-----
//JB TODO: #include <arm_navigation_msgs/MoveArmAction.h>
#include "moveit_msgs/PlanningScene.h"
//#include "moveit_msgs/ApplyPlanningScene.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/AttachedCollisionObject.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "geometry_msgs/Pose.h"
// #include "moveit/move_group_interface/move_group.h"
// #include "moveit/planning_scene_interface/planning_scene_interface.h"
//#include "moveit/move_group/move_group_context.h"
//#include "moveit/planning_interface/move_group.h"
// #include "moveit/move_group_interface/move_group_interface.h"

// #include "~/ws_moveit/src/moveit/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
//-----

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <tf/transform_listener.h>
#include <table_setting_demo/pick_and_place.h>
#include <table_setting_demo/pick_and_place_state.h>
#include <table_setting_demo/pick_and_place_stop.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "table_setting_demo/pick_and_place.h"
#include "actionlib/client/simple_action_client.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "tf/transform_listener.h"
// #include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/pick_and_place_state.h"
#include "table_setting_demo/pick_and_place_stop.h"

namespace pr2 {

typedef enum STATE {
  APPROACHING = 0,
  PICKING,
  PICKED,
  PLACING,
  PLACED,
  NEUTRAL,
  IDLE
} STATE_t;

struct PickPlaceGoal {
//-----
  //JB TODO: arm_navigation_msgs::MoveArmGoal pick_pose;
  //JB TODO: arm_navigation_msgs::MoveArmGoal place_pose;
  geometry_msgs::Pose pick_pose;
  geometry_msgs::Pose place_pose;
//-----
};

typedef struct Point {
  double x,y,z,w;
}__attribute__((packed)) Point_t;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
//-----
//JB TODO: typedef arm_navigation_msgs::MoveArmGoal MoveArmGoal_t;
typedef geometry_msgs::Pose Pose_t;
//-----

class Gripper {
 public:
  Gripper();
  virtual ~Gripper();

  virtual void Open();
  virtual void Close();
 private:
  GripperClient *gripper_client_;
};

class PickPlace {
 public:
  PickPlace(std::string arm);
  virtual ~PickPlace();

  bool PickAndPlaceObject(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  bool PickAndPlacecheck(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  bool PickAndPlaceState(
    table_setting_demo::pick_and_place_state::Request &req,
    table_setting_demo::pick_and_place_state::Response &res);
  bool PickAndPlaceStop(
    table_setting_demo::pick_and_place_stop::Request &req,
    table_setting_demo::pick_and_place_stop::Response &res);
  void PostParameters();
  void CalibrateObjects();
  void ReadCalibration(std::string filename);
//-----
  /* //JB TODO:    MoveArmGoal_t GetArmPoseFromPoints(
    std::string frame_id,
    std::string link,
    Point_t position,
    Point_t orientation); */
  Pose_t GetArmPoseFromPoints(
    std::string frame_id,
    std::string link,
    Point_t position,
    Point_t orientation); 
//-----
  void SaveCalibration(std::string filename);
  void PickAndPlaceImpl(std::string object);

// TODO JB: ADDED FUNCTIONALITY TO CREATE SCENE OBJECTS
  void SetSceneObjects();
  void SetSceneBounds();
  int getIndex(std::string object);

 private:
//-----
  //TODO JB: bool SendGoal(MoveArmGoal_t goal);
  //TODO JB:   MoveArmGoal_t GetArmPoseGoal();
  bool SendGoal(Pose_t goal);
  Pose_t GetArmPoseGoal(); 
//-----

  ros::NodeHandle nh_;
  std::vector<std::string> objects_;
  std::vector<std::string> static_objects_;
  std::vector<std::string> dynamic_objects_;
  std::string arm_;
  std::map<std::string, PickPlaceGoal> object_goal_map_;

//-----
  //JB TODO: actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;
  moveit::planning_interface::MoveGroup arm_group_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
//-----

  Gripper r_gripper_;
  uint32_t state_;
  bool stop;
  boost::shared_ptr<boost::thread> work_thread;
};
}
#endif  // PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE
