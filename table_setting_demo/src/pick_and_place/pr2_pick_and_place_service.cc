#include "pick_and_place/pr2_pick_and_place_service.h"
#include "table_setting_demo/object_position.h"
#include "table_setting_demo/ObjectTransformation.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>
#include "log.h"
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "ros/ros.h"
#include "active_vision_msgs/Vision_Service.h"
#include "active_vision_msgs/Vision_Message.h"

#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


int getch() {
  static termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

namespace pr2 {
//Open the gripper
Gripper::Gripper(){

  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

  //wait for the gripper action server to come up 
  while(!gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action"
             "action server to come up");
  }
}

Gripper::~Gripper(){
  delete gripper_client_;
}

void Gripper::Open(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  // open.command.max_effort = 80.0;  // Do not limit effort (negative)
  open.command.max_effort = -1.0;  // Do not limit effort (negative)
  
  ROS_INFO("Sending open goal");
  gripper_client_->sendGoal(open);
  sleep(5.0);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened!");
  else
    ROS_INFO("The gripper failed to open.");
}

//Close the gripper
void Gripper::Close(){
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  // squeeze.command.max_effort = 30.0;  // Close gently
  squeeze.command.max_effort = -1.0;  // Close gently
  
  ROS_INFO("Sending squeeze goal");
  gripper_client_->sendGoal(squeeze);
  sleep(5.0);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper closed!");
  else
    ROS_INFO("The gripper failed to close.");
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

PickPlace::PickPlace(std::string arm) : arm_group_{"right_arm"}  {
  arm_ = arm;
  // arm_group_.setPlannerId("PRMkConfigDefault");


  const char *dynamic_object_str[] = {
    // "cup",
     //"bowl",
    // "soda",
    // "fork",
    // "spoon",
    // "knife"
    // "Lettuce",
    // "Cup",
    // "Tea",
    // "Sugar", 
    // "Meat",
    // "Right_Bread", 
    // "Left_Bread",
  };
  const char *static_object_str[] = {

    // // "cup",
    // //"bowl",
    // // "soda",
    // "Lettuce",
    // "Cup",
    // "Tea",
    // "Sugar", 
    // "Meat",
    // "Right_Bread", 
    // "Left_Bread",
    // "neutral"
    // //"placemat",
    // // "wineglass",
    // //"plate"
  };
  const char *object_str[] = {
    
  //   // // "placemat",
  //   // // "cup",
  //   // //"plate",
  //   // // "Lettuce",
  //   // // "Cup",
  //   // // "Tea",
  //   // // "Sugar", 
  //   // // "Meat",
  //   // "Right_Bread", 
  //   // "Left_Bread",
  //   // "neutral"
  //   // // "fork",
  //   // // "spoon",
  //   // // "knife",
  //   // //"bowl",
  //   // //"soda",
  //   // // "wineglass"
  // "teddy_bear",
  // // "orange",
  // "sports_ball",
  // "clock",
  // // "bottle",
  // "scissors",
  // "cup",
  // // "bowl"
  "Cup",
  "Tea_Pot",
  //"Sugar",
  "Burger",
  //"Sandwich",
  "Apple",
  //"Orange"
  };
  objects_ = std::vector<std::string>(object_str,
    object_str + sizeof(object_str) / sizeof(char*));

  static_objects_ = std::vector<std::string>(static_object_str,
    static_object_str + sizeof(static_object_str) / sizeof(char*));

  dynamic_objects_ = std::vector<std::string>(dynamic_object_str,
    dynamic_object_str + sizeof(dynamic_object_str) / sizeof(char*));

  for (uint32_t i = 0; i < objects_.size(); ++i) {
    printf("Object: %s\n", objects_[i].c_str());
  } 

  // SET STATE
  arm_group_.setStartStateToCurrentState();
  arm_group_.setPoseReferenceFrame("/odom_combined");
  state_ = IDLE;

  // TODO JB: Create the scene objects!
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // SetSceneObjects();
  // SetSceneBounds();
  
} 

PickPlace::~PickPlace() {}

void PickAndPlaceThread(PickPlace *manipulation, std::string object) {

  // old way of manipulation
  if ( !manipulation->visionManipVer ) {
    manipulation->PickAndPlaceImpl(object);
  }
  // new way of manipulation with vision manip pipeiline
  else {
    manipulation->PickAndPlaceImpl_VisionManip(object);
  }
}

void TransformPoseLocalToWorld(
  geometry_msgs::PoseStamped &input,
  geometry_msgs::PoseStamped &output,
  geometry_msgs::TransformStamped transform) {
  output.pose.position.x = input.pose.position.x + transform.transform.translation.x;
  output.pose.position.y = input.pose.position.y + transform.transform.translation.y;
  output.pose.position.z = input.pose.position.z + transform.transform.translation.z;

  output.pose.orientation = input.pose.orientation;
}

void TransformPoseWorldToLocal(
  geometry_msgs::PoseStamped &input,
  geometry_msgs::PoseStamped &output,
  geometry_msgs::TransformStamped transform) {
  output.pose.position.x = input.pose.position.x - transform.transform.translation.x;
  output.pose.position.y = input.pose.position.y - transform.transform.translation.y;
  output.pose.position.z = input.pose.position.z - transform.transform.translation.z;

  output.pose.orientation = input.pose.orientation;
}

void PickPlace::PickAndPlaceImpl_VisionManip(std::string object) {

  printf("Picking up Object: %s\n", object.c_str());
  geometry_msgs::Pose pick_pose_offset;
  geometry_msgs::Pose approach_pose_offset;
  geometry_msgs::Pose place_pose_offset;
  if (stop)
    return;

  state_ = NEUTRAL;
  r_gripper_.Open();
  state_ = APPROACHING;

   //---------------
  //  Move to neutral place
  ROS_INFO("Goal: %s APPROACH Plus Z", object.c_str());
  //   printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].place_pose.position.x, object_goal_map_["neutral"].place_pose.position.y, object_goal_map_["neutral"].place_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].place_pose.orientation.x, object_goal_map_["neutral"].place_pose.orientation.y, object_goal_map_["neutral"].place_pose.orientation.z, object_goal_map_["neutral"].place_pose.orientation.z);
  approach_pose_offset = object_goal_map_[object.c_str()].approach_pose;
    approach_pose_offset.position.z = std::min(approach_pose_offset.position.z + 0.2, 1.1); 
    approach_pose_offset.position.x = approach_pose_offset.position.x; 
    approach_pose_offset.position.y = approach_pose_offset.position.y; 
  // if (approach_pose_offset.position.z < 1.1 ) {
    printf("\n  goal pos x %f  y %f  z %f \n", approach_pose_offset.position.x, approach_pose_offset.position.y, approach_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", approach_pose_offset.orientation.x, approach_pose_offset.orientation.y, approach_pose_offset.orientation.z, approach_pose_offset.orientation.w);
    if (!SendGoal(approach_pose_offset)) {  
      ROS_INFO("Goal ERROR!!!!");
      // sleep(2.0);
      state_ = PLACED;
      return;
    }
    if (stop)
      return;
    state_ = PICKING;
    ROS_INFO("     State is now PICKING");
  // }

 //---------------
  // Move to Neutral Start
  ROS_INFO("Goal: %s PICK APPROACH", object.c_str());
  // printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].pick_pose.position.x, object_goal_map_["neutral"].pick_pose.position.y, object_goal_map_["neutral"].pick_pose.position.z);
  // printf("\n  goal ori pick place imp x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].pick_pose.orientation.x, object_goal_map_["neutral"].pick_pose.orientation.y, object_goal_map_["neutral"].pick_pose.orientation.z, object_goal_map_["neutral"].pick_pose.orientation.z);
  // ROS_INFO("              planner_id: %s", arm_group_.getDefaultPlannerId(arm_).c_str());

  approach_pose_offset = object_goal_map_[object.c_str()].approach_pose;
  printf("\n  goal pos x %f  y %f  z %f \n", approach_pose_offset.position.x, approach_pose_offset.position.y, approach_pose_offset.position.z);
  printf("\n  goal ori x %f  y %f  z %f  w %f\n", approach_pose_offset.orientation.x, approach_pose_offset.orientation.y, approach_pose_offset.orientation.z, approach_pose_offset.orientation.w);

  if (!SendGoal(approach_pose_offset)) { 
    ROS_INFO("Goal ERROR!!!!");
    // sleep(2.0);
    state_ = PLACED;
    return;
  } 
  if (stop)
    return;
  state_ = PICKING;
  ROS_INFO("     State is now PICKING");

  //---------------
  // Move to Object Pick location
  ROS_INFO("Goal: %s PICK", object.c_str());
  // printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_[object.c_str()].pick_pose.position.x, object_goal_map_[object.c_str()].pick_pose.position.y, object_goal_map_[object.c_str()].pick_pose.position.z);
  // printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_[object.c_str()].pick_pose.orientation.x, object_goal_map_[object.c_str()].pick_pose.orientation.y, object_goal_map_[object.c_str()].pick_pose.orientation.z, object_goal_map_[object.c_str()].pick_pose.orientation.z);
  if (!SendGoal(object_goal_map_[object.c_str()].pick_pose)) {
    ROS_INFO("Goal ERROR!!!!");
    // sleep(2.0);
    state_ = PLACED;
    return;
  }
  if (stop)
  return;
  r_gripper_.Close();
  // TODO JB: attach object to arm using moveit
  ROS_INFO("Attach the object to the robot");
  int index = getIndex(object);
  if (index != -1) {
  arm_group_.attachObject(collision_objects_[index].id);
  }
  /* Sleep to give Rviz time to show the object attached (different color). */
  // sleep(4.0);
  state_ = PICKED;
  ROS_INFO("     State is now PICKED");
  // Move to Neutral start
  if (stop)
    return;

  //---------------
  //  Move to neutral place
  ROS_INFO("Goal: %s PICK Plus Z", object.c_str());
  //   printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].place_pose.position.x, object_goal_map_["neutral"].place_pose.position.y, object_goal_map_["neutral"].place_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].place_pose.orientation.x, object_goal_map_["neutral"].place_pose.orientation.y, object_goal_map_["neutral"].place_pose.orientation.z, object_goal_map_["neutral"].place_pose.orientation.z);
  pick_pose_offset = object_goal_map_[object.c_str()].pick_pose;
    pick_pose_offset.position.z = std::min(pick_pose_offset.position.z + 0.2, 1.1); 
    pick_pose_offset.position.x = pick_pose_offset.position.x; 
    pick_pose_offset.position.y = pick_pose_offset.position.y; 
    printf("\n  goal pos x %f  y %f  z %f \n", pick_pose_offset.position.x, pick_pose_offset.position.y, pick_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", pick_pose_offset.orientation.x, pick_pose_offset.orientation.y, pick_pose_offset.orientation.z, pick_pose_offset.orientation.w);
  if (!SendGoal(pick_pose_offset)) {  
    ROS_INFO("Goal ERROR!!!!");
    // sleep(2.0);
    state_ = PLACED;
    return;
  }
  if (stop)
    return;
  // !!! Don't want to move back to pick as unncessesary with +z instead of neutral
  state_ = PLACING;
  ROS_INFO("     State is now PLACING");

  //---------------
  //  Move to neutral place
  ROS_INFO("Goal: %s PLACE Plus Z", object.c_str());
  //   printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].place_pose.position.x, object_goal_map_["neutral"].place_pose.position.y, object_goal_map_["neutral"].place_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].place_pose.orientation.x, object_goal_map_["neutral"].place_pose.orientation.y, object_goal_map_["neutral"].place_pose.orientation.z, object_goal_map_["neutral"].place_pose.orientation.z);
  place_pose_offset = object_goal_map_[object.c_str()].place_pose;
    place_pose_offset.position.z = std::min(place_pose_offset.position.z + 0.2, 1.1); 
    place_pose_offset.position.x = place_pose_offset.position.x - 0.1; 
    place_pose_offset.position.y = place_pose_offset.position.y - 0.1; 

    printf("\n  goal pos x %f  y %f  z %f \n", place_pose_offset.position.x, place_pose_offset.position.y, place_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", place_pose_offset.orientation.x, place_pose_offset.orientation.y, place_pose_offset.orientation.z, place_pose_offset.orientation.w);
  if (!SendGoal(place_pose_offset)) {
    ROS_INFO("Goal ERROR!!!!");
    // sleep(2.0);
    state_ = PLACED;
    return;
  }
  if (stop)
    return;

  //---------------
  //  Move to object place
  ROS_INFO("Goal: %s PLACE", object.c_str());
    printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_[object.c_str()].place_pose.position.x, object_goal_map_[object.c_str()].place_pose.position.y, object_goal_map_[object.c_str()].place_pose.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_[object.c_str()].place_pose.orientation.x, object_goal_map_[object.c_str()].place_pose.orientation.y, object_goal_map_[object.c_str()].place_pose.orientation.z, object_goal_map_[object.c_str()].place_pose.orientation.z);
  if (!SendGoal(object_goal_map_[object.c_str()].place_pose)) {
    ROS_INFO("Goal ERROR!!!!");
    // sleep(2.0);
    state_ = PLACED;
    return;
  }
  if (stop)
    return;
  r_gripper_.Open();
  // TODO JB: dettach object to arm using moveit
  ROS_INFO("Detach the object from the robot");
  index = getIndex(object);
  if (index != -1) {
  arm_group_.detachObject(collision_objects_[index].id);
  }
  /* Sleep to give Rviz time to show the object detached. */
  // sleep(4.0);
  // The pickandplacecheck uses this to tell when done, so it should be moved to after reset to neutral pick
  // state_ = PLACED;
  // ROS_INFO("     State is now PLACED");
  if (stop)
    return;

  //---------------
  //  Move to neutral place
  ROS_INFO("Goal: %s PLACE Plus Z", object.c_str());
    printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].place_pose.position.x, object_goal_map_["neutral"].place_pose.position.y, object_goal_map_["neutral"].place_pose.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].place_pose.orientation.x, object_goal_map_["neutral"].place_pose.orientation.y, object_goal_map_["neutral"].place_pose.orientation.z, object_goal_map_["neutral"].place_pose.orientation.z);
  place_pose_offset = object_goal_map_[object.c_str()].place_pose;
    place_pose_offset.position.z = std::min(place_pose_offset.position.z + 0.2, 1.1); 
    place_pose_offset.position.x = place_pose_offset.position.x - 0.1; 
    place_pose_offset.position.y = place_pose_offset.position.y - 0.1; 
    printf("\n  goal pos x %f  y %f  z %f \n", place_pose_offset.position.x, place_pose_offset.position.y, place_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", place_pose_offset.orientation.x, place_pose_offset.orientation.y, place_pose_offset.orientation.z, place_pose_offset.orientation.w);
  if (!SendGoal(place_pose_offset)) {  
    ROS_INFO("Goal ERROR!!!!");
    // sleep(2.0);
    state_ = PLACED;
    return;
  }
  if (stop)
    return;
  // !!! Don't want to move back to pick as unncessesary with +z instead of neutral
  state_ = PLACED;
  ROS_INFO("     State is now PLACED");

}


void PickPlace::PickAndPlaceImpl(std::string object) {
  printf("Picking up Object: %s\n", object.c_str());
  geometry_msgs::Pose pick_pose_offset;
  geometry_msgs::Pose place_pose_offset;

  // check if dyanamic or static object
  if (stop)
    return;
  table_setting_demo::object_position pos_msg;
  table_setting_demo::ObjectTransformation pose_msg;
  bool dynamic = true;
  for (int i = 0; i < static_objects_.size(); ++i) {
    if (object == static_objects_[i]) {
      dynamic = false;
      break;
    }
  }
  if (dynamic) {
    ROS_INFO("Object Is dynamic");
  } else {
    ROS_INFO("Object Is Static");
  }
  state_ = NEUTRAL;
  r_gripper_.Open();
  state_ = APPROACHING;
  // Move to Neutral Start
//------
  //TODO JB: object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  // N/A?!  
//------
  
  if (stop)
    return;
//------
  // Todo goes through here -> goal is object_goal_map_["neutral"].pick_pose



  ROS_INFO("Goal is Neutral PICK");
  // printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].pick_pose.position.x, object_goal_map_["neutral"].pick_pose.position.y, object_goal_map_["neutral"].pick_pose.position.z);
  // printf("\n  goal ori pick place imp x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].pick_pose.orientation.x, object_goal_map_["neutral"].pick_pose.orientation.y, object_goal_map_["neutral"].pick_pose.orientation.z, object_goal_map_["neutral"].pick_pose.orientation.z);
  // ROS_INFO("              planner_id: %s", arm_group_.getDefaultPlannerId(arm_).c_str());

  //if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
  pick_pose_offset = object_goal_map_[object.c_str()].pick_pose;
  pick_pose_offset.position.z = pick_pose_offset.position.z + 0.2; 
  pick_pose_offset.position.x = pick_pose_offset.position.x - 0.1; 
  pick_pose_offset.position.y = pick_pose_offset.position.y - 0.1; 
  printf("\n  goal pos x %f  y %f  z %f \n", pick_pose_offset.position.x, pick_pose_offset.position.y, pick_pose_offset.position.z);
  printf("\n  goal ori x %f  y %f  z %f  w %f\n", pick_pose_offset.orientation.x, pick_pose_offset.orientation.y, pick_pose_offset.orientation.z, pick_pose_offset.orientation.w);

  if (!SendGoal(pick_pose_offset)) {
    ROS_INFO("Goal ERROR!!!!");
    return;
  } 
//------
  if (stop)
    return;
  // Move to Object Pick location

  // Check if Object is dynamic or static
  if (dynamic) {
    // request object tracked position
 //    pos_msg.request.object_id = object;
 //    if (!ros::service::call("vision_service", pos_msg)) {
 //      ROS_ERROR("Service: [%s] not available!", "Vision service");
 //      //to-do: santosh: change to get vision service to get location of one object.
 //      pose_msg.response.transform.transform.translation.x = 0;
 //      pose_msg.response.transform.transform.translation.y = 0;
 //      pose_msg.response.transform.transform.translation.z = 0;
 //    }
 // else {
 //      pose_msg.response.transform.transform.translation.x = 0;
 //      pose_msg.response.transform.transform.translation.y = 0;
 //      pose_msg.response.transform.transform.translation.z = 0;
 //    }


//---------------------------------------------------------------------------------------
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<active_vision_msgs::Vision_Service>("/CV_Objects");
    active_vision_msgs::Vision_Service srv;

    tf::TransformListener listener;
    geometry_msgs::PointStamped kinect_obj_pnt;
    geometry_msgs::PointStamped world_obj_pnt;
   
    srv.request.Label = object;
    ROS_INFO("Just before service :-(");
    if (client.call(srv))
    {
      ROS_INFO("Being served!!!");
      //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      active_vision_msgs::Vision_Message msg;
      msg = srv.response.object_location[0];
      ROS_INFO("Frame id is %s", msg.Frameid.c_str());
      ROS_INFO("x = %f", msg.Pos.x);
      ROS_INFO("y = %f", msg.Pos.y);
      ROS_INFO("z = %f", msg.Pos.z);
      ROS_INFO("Is available: %u", msg.Found);
      if (msg.Found == true)
      {
        kinect_obj_pnt.header.frame_id = "head_mount_kinect_rgb_optical_frame";//msg.Frameid;
        kinect_obj_pnt.header.stamp = ros::Time();
        kinect_obj_pnt.point.x = msg.Pos.x;
        kinect_obj_pnt.point.y = msg.Pos.y;
        kinect_obj_pnt.point.z = msg.Pos.z;

        try
        {
          ROS_INFO("Waiting for Tf");
          listener.waitForTransform("torso_lift_link", "head_mount_kinect_rgb_link", ros::Time::now(), ros::Duration(5.0));
          listener.transformPoint("torso_lift_link", kinect_obj_pnt, world_obj_pnt);
          ROS_INFO("world x = %f", world_obj_pnt.point.x);
          ROS_INFO("world y = %f", world_obj_pnt.point.y);
          ROS_INFO("world z = %f", world_obj_pnt.point.z);
          ROS_INFO("============= transformed");
        }
        catch(tf::TransformException& ex)
        {
          ROS_ERROR("Received an exception while trying to transform the point");
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to call service vision_server");
      //return 1;
    }

    // Transform pose into world space
    geometry_msgs::PoseStamped object_pose, world_pose;
//------
    //JB TODO: arm_navigation_msgs::MoveArmGoal pick_pose = object_goal_map_[object.c_str()].pick_pose;
    geometry_msgs::Pose pick_pose = object_goal_map_[object.c_str()].pick_pose;
    ROS_INFO("     Pick_pose set to goal map pose");


    //TODO JB: object_pose.pose.position = pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position;
    //TODO JB: object_pose.pose.orientation = pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;
    object_pose.pose.position = pick_pose.position;
    object_pose.pose.orientation = pick_pose.orientation;
    ROS_INFO("     Object pose set to pick_pose");
//------

    TransformPoseLocalToWorld(object_pose, world_pose, pose_msg.response.transform);
    ROS_INFO("     pose tramsformed from local to world");


    state_ = PICKING;
    ROS_INFO("     State is now PICKING");

//------
    //TODO JB: pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position = world_pose.pose.position;
    //TODO JB: pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation = world_pose.pose.orientation;
    pick_pose.position = world_pose.pose.position;
    pick_pose.orientation = world_pose.pose.orientation;
    // ROS_INFO("     Pick_pose set to world pose");


//------
    if (stop)
    return;
    ROS_INFO("Goal: %s PICK", object.c_str());
    printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_[object.c_str()].pick_pose.position.x, object_goal_map_[object.c_str()].pick_pose.position.y, object_goal_map_[object.c_str()].pick_pose.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_[object.c_str()].pick_pose.orientation.x, object_goal_map_[object.c_str()].pick_pose.orientation.y, object_goal_map_[object.c_str()].pick_pose.orientation.z, object_goal_map_[object.c_str()].pick_pose.orientation.z);

    if (!SendGoal(pick_pose)) {
      return;
    }
    if (stop)
    return;
  }
  // if not dynamic, do the following
  else {
    if (stop)
    return;
    //TODO JB: object_goal_map_[object.c_str()].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
    ROS_INFO("Goal: %s PICK", object.c_str());
    // printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_[object.c_str()].pick_pose.position.x, object_goal_map_[object.c_str()].pick_pose.position.y, object_goal_map_[object.c_str()].pick_pose.position.z);
    // printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_[object.c_str()].pick_pose.orientation.x, object_goal_map_[object.c_str()].pick_pose.orientation.y, object_goal_map_[object.c_str()].pick_pose.orientation.z, object_goal_map_[object.c_str()].pick_pose.orientation.z);
    if (!SendGoal(object_goal_map_[object.c_str()].pick_pose)) {
      ROS_INFO("Goal ERROR!!!!");
      return;
    }
    if (stop)
    return;
  }
  r_gripper_.Close();
  // TODO JB: attach object to arm using moveit
  ROS_INFO("Attach the object to the robot");
  int index = getIndex(object);
  if (index != -1) {
  arm_group_.attachObject(collision_objects_[index].id);
  }
  /* Sleep to give Rviz time to show the object attached (different color). */
  // sleep(4.0);
  state_ = PICKED;
  ROS_INFO("     State is now PICKED");
  // Move to Neutral start
  if (stop)
    return;
  //TODO JB: object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();

  ROS_INFO("Goal: Neutral PICK");
  //     printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].pick_pose.position.x, object_goal_map_["neutral"].pick_pose.position.y, object_goal_map_["neutral"].pick_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].pick_pose.orientation.x, object_goal_map_["neutral"].pick_pose.orientation.y, object_goal_map_["neutral"].pick_pose.orientation.z, object_goal_map_["neutral"].pick_pose.orientation.z);
  // if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    pick_pose_offset = object_goal_map_[object.c_str()].pick_pose;
    pick_pose_offset.position.z = pick_pose_offset.position.z + 0.2; 
    pick_pose_offset.position.x = pick_pose_offset.position.x - 0.1; 
    pick_pose_offset.position.y = pick_pose_offset.position.y - 0.1; 

    printf("\n  goal pos x %f  y %f  z %f \n", pick_pose_offset.position.x, pick_pose_offset.position.y, pick_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", pick_pose_offset.orientation.x, pick_pose_offset.orientation.y, pick_pose_offset.orientation.z, pick_pose_offset.orientation.w);
  if (!SendGoal(pick_pose_offset)) {
    ROS_INFO("Goal ERROR!!!!");
    ROS_INFO("Detach the object from the robot");
    index = getIndex(object);
    if (index != -1) {
    arm_group_.detachObject(collision_objects_[index].id);
    }
    return;
  }
  if (stop)
    return;
  state_ = PLACING;
  ROS_INFO("     State is now PLACING");
  //TODO JB: object_goal_map_["neutral"].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();

  ROS_INFO("Goal: Neutral PLACE");
  //   printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].place_pose.position.x, object_goal_map_["neutral"].place_pose.position.y, object_goal_map_["neutral"].place_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].place_pose.orientation.x, object_goal_map_["neutral"].place_pose.orientation.y, object_goal_map_["neutral"].place_pose.orientation.z, object_goal_map_["neutral"].place_pose.orientation.z);
  // if (!SendGoal(object_goal_map_["neutral"].place_pose)) {
  place_pose_offset = object_goal_map_[object.c_str()].place_pose;
    place_pose_offset.position.z = place_pose_offset.position.z + 0.2; 
    place_pose_offset.position.x = place_pose_offset.position.x - 0.1; 
    place_pose_offset.position.y = place_pose_offset.position.y - 0.1; 

    printf("\n  goal pos x %f  y %f  z %f \n", place_pose_offset.position.x, place_pose_offset.position.y, place_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", place_pose_offset.orientation.x, place_pose_offset.orientation.y, place_pose_offset.orientation.z, place_pose_offset.orientation.w);
  if (!SendGoal(place_pose_offset)) {
    ROS_INFO("Goal ERROR!!!!");
    return;
  }
  if (stop)
    return;

  // object place
  //TODO JB: object_goal_map_[object.c_str()].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  ROS_INFO("Goal: %s PLACE", object.c_str());
    printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_[object.c_str()].place_pose.position.x, object_goal_map_[object.c_str()].place_pose.position.y, object_goal_map_[object.c_str()].place_pose.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_[object.c_str()].place_pose.orientation.x, object_goal_map_[object.c_str()].place_pose.orientation.y, object_goal_map_[object.c_str()].place_pose.orientation.z, object_goal_map_[object.c_str()].place_pose.orientation.z);
  if (!SendGoal(object_goal_map_[object.c_str()].place_pose)) {
    ROS_INFO("Goal ERROR!!!!");
    return;
  }
  if (stop)
    return;
  r_gripper_.Open();
  // TODO JB: dettach object to arm using moveit
  ROS_INFO("Detach the object from the robot");
  index = getIndex(object);
  if (index != -1) {
  arm_group_.detachObject(collision_objects_[index].id);
  }
  /* Sleep to give Rviz time to show the object detached. */
  // sleep(4.0);
  // The pickandplacecheck uses this to tell when done, so it should be moved to after reset to neutral pick
  // state_ = PLACED;
  // ROS_INFO("     State is now PLACED");

  if (stop)
    return;
   //TODO JB: object_goal_map_["neutral"].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();

  ROS_INFO("Goal: Neutral PLACE");
  //   printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].place_pose.position.x, object_goal_map_["neutral"].place_pose.position.y, object_goal_map_["neutral"].place_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].place_pose.orientation.x, object_goal_map_["neutral"].place_pose.orientation.y, object_goal_map_["neutral"].place_pose.orientation.z, object_goal_map_["neutral"].place_pose.orientation.z);
  // if (!SendGoal(object_goal_map_["neutral"].place_pose)) {
  place_pose_offset = object_goal_map_[object.c_str()].place_pose;
    place_pose_offset.position.z = place_pose_offset.position.z + 0.2; 
    place_pose_offset.position.x = place_pose_offset.position.x - 0.1; 
    place_pose_offset.position.y = place_pose_offset.position.y - 0.1; 
    printf("\n  goal pos x %f  y %f  z %f \n", place_pose_offset.position.x, place_pose_offset.position.y, place_pose_offset.position.z);
    printf("\n  goal ori x %f  y %f  z %f  w %f\n", place_pose_offset.orientation.x, place_pose_offset.orientation.y, place_pose_offset.orientation.z, place_pose_offset.orientation.w);
  if (!SendGoal(place_pose_offset)) {  

    ROS_INFO("Goal ERROR!!!!");
    return;
  }

  if (stop)
    return;
  // !!! Don't want to move back to pick as unncessesary with +z instead of neutral

  //TODO JB: object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  // ROS_INFO("Goal: Neutral PICK");
  //   printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].pick_pose.position.x, object_goal_map_["neutral"].pick_pose.position.y, object_goal_map_["neutral"].pick_pose.position.z);
  //   printf("\n  goal ori x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].pick_pose.orientation.x, object_goal_map_["neutral"].pick_pose.orientation.y, object_goal_map_["neutral"].pick_pose.orientation.z, object_goal_map_["neutral"].pick_pose.orientation.z);
  // if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {

  //   ROS_INFO("Goal ERROR!!!!");
  //   return;
  // }
  // since check uses this to verify done, put this here!
  state_ = PLACED;
  ROS_INFO("     State is now PLACED");

}

bool PickPlace::PickAndPlaceObject(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res) {
  stop = true;
  ROS_INFO("PickPlace::PickAndPlaceObject => stop == true");
  if (work_thread) {
    ROS_INFO("PickPlace::PickAndPlaceObject => work thread exists, so join it");  
    work_thread->join(); }
  stop = false;
  ROS_INFO("PickPlace::PickAndPlaceObject => stop == false");
  work_thread =  boost::shared_ptr<boost::thread>(new boost::thread(&PickAndPlaceThread, this, req.object));
  res.success = true;
  ROS_INFO("PickPlace::PickAndPlaceObject => get work_thread and set res.success = true");
  return true;
}

bool PickPlace::PickAndPlacecheck(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res) {
  // Check if Object is correct
  // check if Pick and Place is Done
  res.success = (state_ == PLACED);
  if (state_ == PLACED) {
    state_ = IDLE;
  }
  return true;
}

bool PickPlace::PickAndPlaceState(
    table_setting_demo::pick_and_place_state::Request &req,
    table_setting_demo::pick_and_place_state::Response &res) {
  res.state = state_;
  return true;
}

bool PickPlace::PickAndPlaceStop(
    table_setting_demo::pick_and_place_stop::Request &req,
    table_setting_demo::pick_and_place_stop::Response &res) {
  stop = true;
  //TODO JB: move_arm_.cancelGoal();
  arm_group_.stop();
  work_thread->join();
  stop = false;
  return true;
}

void PickPlace::PostParameters() {
  std::vector<double> positions;
  std::string topic = "/ObjectPositions/";
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) {
    //TODO JB: positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x);
    //TODO JB: positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y);
    //TODO JB: positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z);
    positions.push_back(it->second.pick_pose.position.x);
    positions.push_back(it->second.pick_pose.position.y);
    positions.push_back(it->second.pick_pose.position.z);
    ros::param::set((topic + it->first).c_str(), positions);
    positions.clear();
  }
}

void waitKeyboard() {
  while (true) {
    int c = getch();
    if (c == ' ')
      break;
    if (c == 10)
      break;
  }
}

bool waitKeyboardYesNo() {
  bool confirm = true;
  while (true) {
    int c = getch();
    if (c == ' ')
      break;
    if (c == 10)
      break;
    if (c == 'n' || c == 'N')
      confirm = false;
  }
  return confirm;
}

void PickPlace::OnlineDetectionsPlaces() {

  r_gripper_.Open();
  for (uint32_t i = 0; i < objects_.size(); ++i) {
    printf(
      "Place the [%s] into the [%s] gripper and Press Enter\n",
      objects_[i].c_str(),
      arm_.c_str());
    waitKeyboard();
    r_gripper_.Close();
    printf(
      "Move [%s] limb to: [%s] Placeing location and Press Enter\n",
      arm_.c_str(),
      objects_[i].c_str());
    waitKeyboard();

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", arm_group_.getPlanningFrame().c_str());
 
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", arm_group_.getEndEffectorLink().c_str());

  // JB WAY
    geometry_msgs::PoseStamped currentPose;
    currentPose = arm_group_.getCurrentPose();

  std::cout << "Current pose: " << currentPose << '\n';

  // LUKE WAY
    currentPose.pose = GetArmPoseGoal();
  std::cout << "Current pose: " << currentPose << '\n';


    object_goal_map_[objects_[i]].place_pose = currentPose.pose; //TODO JB_INTEGRATION verify this is in the right frame!!!
    r_gripper_.Open();
  }

}

void PickPlace::OnlineDetectionsPicks( ros::ServiceClient *visManipClient_pntr ) {

  // ros::ServiceClient visManipClient = n.serviceClient<vision_manip_pipeline::VisionManip>("vision_manip");
  for (uint32_t i = 0; i < objects_.size(); ++i) {
    // int res = visionManipPipeline( objects_[i], nh);  //TODO JB_INTEGRATION replace this call with vision manup pipeline
                                                      //  THE OUTPUT IS COMPLETELY WRONG FOR THIS FUNCTION, FIXXXXXXX TO BE POSE
    // system(" sudo invoke-rc.d chrony restart");
    // sleep(3);
    vision_manip_pipeline::VisionManip visManipSrv;
    visManipSrv.request.obj_name = objects_[i].c_str();
    if(visManipClient_pntr->call(visManipSrv)){
      // ROS_INFO("NewX: %f NewY: %f NewZ: %f", (float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
      std::cout << "Object:   " << objects_[i].c_str() << '\n';
      std::cout << "Approach Pose:   " << visManipSrv.response.approach_pose << '\n';
      std::cout << "Pick Pose:       " << visManipSrv.response.pick_pose << '\n';
      std::cout << "Score of Grasp:  " << visManipSrv.response.score << '\n';
      std::cout << "Top Valid Grasp: " << visManipSrv.response.grasp << '\n';
    }
    else{
      ROS_ERROR("Failed to call service vision_manip, setting score to 0 for object: %s.", objects_[i].c_str());
      // return 1;
    }

    // set the pick pose
    object_goal_map_[objects_[i]].pick_pose = visManipSrv.response.pick_pose.pose; 
    object_goal_map_[objects_[i]].approach_pose = visManipSrv.response.approach_pose.pose; 
    // TODO JB_INTEGRATION: Need to add in the approach pose as it's own thing too instead of doing hardcoded offset as before?!?!?

    // TODO: backup testing - remove when pipleine works!!!
    // geometry_msgs::PoseStamped currentPose;
    // currentPose = arm_group_.getCurrentPose();
    // object_goal_map_[objects_[i]].pick_pose = currentPose.pose; //TODO REPLACE WITH POES FROM VISIONMANIP FUNCTION!!!

   }

}


void PickPlace::CalibrateObjects() {
  char c;
  r_gripper_.Open();
  bool prior_scene_view = false;
  // Get all dynamic object position with clean view
  printf("Would you like to pre-calibrate dynamic object positions?[Y/n]\n");
  prior_scene_view = waitKeyboardYesNo();
  std::map< std::string, geometry_msgs::Transform> dynamic_transforms;
  if (prior_scene_view) {
    // record scene object positions
    printf("Remove arm from scene!\n");
    waitKeyboard();
    table_setting_demo::object_position pos_msg;
    table_setting_demo::ObjectTransformation pose_msg;

    for (int i = 0; i < dynamic_objects_.size(); ++i) {
      pos_msg.request.object_id = dynamic_objects_[i];
      if (!ros::service::call("qr_get_object_position", pos_msg)) {
        ROS_ERROR("ERROR: Service [%s] no available!", "qr_get_object_position");
      }
      if (pos_msg.response.position.size() > 0) {
        pose_msg.request.x = pos_msg.response.position[0];
        pose_msg.request.y = pos_msg.response.position[1];
        pose_msg.request.w = pos_msg.response.position[2];
        pose_msg.request.h = pos_msg.response.position[3];
        pose_msg.request.object = dynamic_objects_[i];

        if (!ros::service::call("object_transformation", pose_msg)) {
          ROS_ERROR("Service: [%s] not available!", "object_transformation");
        }
      } else {
        pose_msg.response.transform.transform.translation.x = 0;
        pose_msg.response.transform.transform.translation.y = 0;
        pose_msg.response.transform.transform.translation.z = 0;
      }

      // store transformation to a list of dynamic_object transformations
      dynamic_transforms[dynamic_objects_[i]] = pose_msg.response.transform.transform;
    }
  }

  for (uint32_t i = 0; i < objects_.size(); ++i) {
    
    bool dynamic = true;
    for (int j = 0; j < static_objects_.size(); ++j) {
      if (objects_[i] == static_objects_[j]) {
        dynamic = false;
        break;
      }
    }

    if (!dynamic || prior_scene_view) {
      printf(
        "Move [%s] limb to: [%s] picking location and Press Enter\n",
        arm_.c_str(),
        objects_[i].c_str());
      waitKeyboard();

      object_goal_map_[objects_[i]].pick_pose = GetArmPoseGoal();


  // printf("  goal pos x %f  y %f  z %f ", object_goal_map_[objects_[i]].pick_pose.position.x, object_goal_map_[objects_[i]].pick_pose.position.y, object_goal_map_[objects_[i]].pick_pose.position.z);
  // printf("  goal ori prior scene view and not dynamic x %f  y %f  z %f  w %f", object_goal_map_[objects_[i]].pick_pose.orientation.x, object_goal_map_[objects_[i]].pick_pose.orientation.y, object_goal_map_[objects_[i]].pick_pose.orientation.z, object_goal_map_[objects_[i]].pick_pose.orientation.z);

    } else {
      printf("Dyanamic Object [%s]! Move arm out of Kinect path! Then Press enter\n", objects_[i].c_str());
      waitKeyboard();
    }
    // check if the object is dynamic
    if (dynamic) {
      table_setting_demo::object_position pos_msg;
      table_setting_demo::ObjectTransformation pose_msg;
      if (!prior_scene_view) {
        // transform into object space
        pos_msg.request.object_id = objects_[i];
        if (!ros::service::call("qr_get_object_position", pos_msg)) {
          ROS_ERROR("Service: [%s] not available!", "qr_get_object_position");
        }
        // Request object 3D transform
        if (pos_msg.response.position.size() > 0) {
          pose_msg.request.x = pos_msg.response.position[0];
          pose_msg.request.y = pos_msg.response.position[1];
          pose_msg.request.w = pos_msg.response.position[2];
          pose_msg.request.h = pos_msg.response.position[3];
          pose_msg.request.object = objects_[i];

          if (!ros::service::call("object_transformation", pose_msg)) {
            ROS_ERROR("Service: [%s] not available!", "object_transformation");
          }
        } else {
          pose_msg.response.transform.transform.translation.x = 0;
          pose_msg.response.transform.transform.translation.y = 0;
          pose_msg.response.transform.transform.translation.z = 0;
        }
      } else {
        pose_msg.response.transform.transform = dynamic_transforms[objects_[i]];
      }
      if (!prior_scene_view) {
        printf(
          "Move [%s] limb to: [%s] picking location and Press Enter\n",
          arm_.c_str(),
          objects_[i].c_str());
        waitKeyboard();
        ROS_INFO("HERE!!!!!");
        object_goal_map_[objects_[i]].pick_pose = GetArmPoseGoal();

  // printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_[objects_[i]].pick_pose.position.x, object_goal_map_[objects_[i]].pick_pose.position.y, object_goal_map_[objects_[i]].pick_pose.position.z);
  // printf("\n  goal ori here x %f  y %f  z %f  w %f\n", object_goal_map_[objects_[i]].pick_pose.orientation.x, object_goal_map_[objects_[i]].pick_pose.orientation.y, object_goal_map_[objects_[i]].pick_pose.orientation.z, object_goal_map_[objects_[i]].pick_pose.orientation.z);
      }

      geometry_msgs::PoseStamped world_pose, object_pose;
      //TODO JB: world_pose.pose.position =    object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position;
      //TODO JB: world_pose.pose.orientation = object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;
      world_pose.pose.position =    object_goal_map_[objects_[i]].pick_pose.position;
      world_pose.pose.orientation = object_goal_map_[objects_[i]].pick_pose.orientation;

      // Transform pose
      TransformPoseWorldToLocal(world_pose, object_pose, pose_msg.response.transform);
      // apply transform
      //TODO JB: object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position = object_pose.pose.position;
      //TODO JB: object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation = object_pose.pose.orientation;
      object_goal_map_[objects_[i]].pick_pose.position = object_pose.pose.position;
      object_goal_map_[objects_[i]].pick_pose.orientation = object_pose.pose.orientation;
    }

    r_gripper_.Close();
    printf(
      "Move [%s] limb to: [%s] Placeing location and Press Enter\n",
      arm_.c_str(),
      objects_[i].c_str());
    waitKeyboard();
    object_goal_map_[objects_[i]].place_pose = GetArmPoseGoal();
    r_gripper_.Open();
  }
}

// void PickPlace::ReadCalibration(std::string filename) {
//   std::ifstream fin;
//   fin.open(filename.c_str(), std::ifstream::binary);
//   char header[256];
//   char frame_id_str[256],link_str[256]; 
//   std::string key;
//   Point_t position, orientation;
//   // Read header
//   fin.read(header, 256);
//   std::string frame_id, link;
//   sscanf(header,"%s\n%s\n", frame_id_str, link_str);
//   frame_id = frame_id_str;
//   link = link_str;
//   // read in
//   while (fin.peek() != EOF) {
//     fin.read(header, 128);
//     key = header;
//     fin.read(reinterpret_cast<char*>(&position), sizeof(Point_t));
//     printf("Position: x: %f, y: %f, z: %f, \n", position.x, position.y, position.z);
//     fin.read(reinterpret_cast<char*>(&orientation), sizeof(Point_t));
//     printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", orientation.x, orientation.y, orientation.z, orientation.w);
//     //TODO JB: object_goal_map_[key].pick_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
//     object_goal_map_[key].pick_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
//     fin.read(reinterpret_cast<char*>(&position), sizeof(Point_t));
//     printf("Position: x: %f, y: %f, z: %f, \n", position.x, position.y, position.z);
//     fin.read(reinterpret_cast<char*>(&orientation), sizeof(Point_t));
//     printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", orientation.x, orientation.y, orientation.z, orientation.w);
//     //TODO JB: object_goal_map_[key].place_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
//     object_goal_map_[key].place_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
//   }
//   fin.close();
// }


void PickPlace::ReadPlaces(std::string filename) 
{
  std::string header;
  std::string frame_id_str;
  std::string frame_id;
  std::string link_str;
  std::string link;
  std::string temp;
  std::string key;
  double val;
  Point_t position, orientation;
  std::stringstream convert;
  std::ifstream infile;
  infile.open(filename);

  getline(infile,frame_id_str);
  getline(infile,link_str);

  link = link_str;
  frame_id = frame_id_str;

  while(infile >> header)
  {
    key = header;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.x = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.y = val;
  
    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.z = val;

    ROS_INFO("%s: %f %f %f", header.c_str(), position.x, position.y, position.z);

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.x = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.y = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.z= val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.w = val;

    ROS_INFO("%s: %f %f %f %f", header.c_str(), orientation.x, orientation.y, orientation.z, orientation.w);
    object_goal_map_[key].place_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);

    convert.str("");
    convert.clear();

  }
}


void PickPlace::ReadCalibration(std::string filename) 
{
  std::string header;
  std::string frame_id_str;
  std::string frame_id;
  std::string link_str;
  std::string link;
  std::string temp;
  std::string key;
  double val;
  Point_t position, orientation;
  std::stringstream convert;
  std::ifstream infile;
  infile.open(filename);

  getline(infile,frame_id_str);
  getline(infile,link_str);

  link = link_str;
  frame_id = frame_id_str;

  while(infile >> header)
  {
    key = header;

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.x = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.y = val;
  
    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.z = val;

    ROS_INFO("%s: %f %f %f", header.c_str(), position.x, position.y, position.z);

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.x= val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.y = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.z= val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.w = val;

    ROS_INFO("%s: %f %f %f %f", header.c_str(), orientation.x, orientation.y, orientation.z, orientation.w);
    object_goal_map_[key].pick_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.x = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.y = val;
  
    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    position.z = val;

    ROS_INFO("%s: %f %f %f", header.c_str(), position.x, position.y, position.z);

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.x= val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.y = val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.z= val;

    convert.str("");
    convert.clear();

    getline(infile,temp,',');
    convert << temp;
    convert >> val;
    orientation.w = val;

    ROS_INFO("%s: %f %f %f %f", header.c_str(), orientation.x, orientation.y, orientation.z, orientation.w);
    object_goal_map_[key].place_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);

    convert.str("");
    convert.clear();

  }
}

geometry_msgs::Pose PickPlace::GetArmPoseFromPoints(std::string frame_id, std::string link, Point_t position, Point_t orientation) {
   geometry_msgs::Pose goal;

  // goal.motion_plan_request.group_name = arm_.c_str();
  // goal.motion_plan_request.num_planning_attempts = 5;
  // goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  arm_group_.setNumPlanningAttempts(15);
  arm_group_.setPlanningTime(15.0);

  // nh_.param<std::string>(
  //   "planner_id",
  //   goal.motion_plan_request.planner_id,
  //   std::string(""));
  // nh_.param<std::string>(
  //   "planner_service_name",
  //   goal.planner_service_name,
  //   std::string("ompl_planning/plan_kinematic_path"));

  // // Setup position of Joint
  // goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  // goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = frame_id.c_str();
  // goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = link.c_str();
  goal.position.x = position.x;
  goal.position.y = position.y;
  goal.position.z = position.z;

  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;
  arm_group_.setGoalPositionTolerance(0.003); 

  // // Setup Orientation
  // goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = frame_id.c_str();
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = link.c_str();

  goal.orientation.x = orientation.x;
  goal.orientation.y = orientation.y;
  goal.orientation.z = orientation.z;
  goal.orientation.w = orientation.w;

  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;
  arm_group_.setGoalOrientationTolerance(0.0075);

  return goal;
} 

//-------


// void PickPlace::SaveCalibration(std::string filename) {
//   std::ofstream fout;
//   fout.open(filename.c_str(), std::ofstream::binary);
//   char header[256];
//   // Save links to first 
//   snprintf(header, 256, "%s\n%s\n", "torso_lift_link", "r_wrist_roll_link");
//   fout.write(header, 256);
//   // save positions
//   Point_t point;
//   point.w = 1;
//   for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
//       it != object_goal_map_.end();
//       ++it) {
//     snprintf(header, 128, "%s", it->first.c_str());
//     fout.write(header, 128);
//     printf("Key:%s\n", header);

//     point.x = it->second.pick_pose.position.x;
//     point.y = it->second.pick_pose.position.y;
//     point.z = it->second.pick_pose.position.z;
//     printf("Position: x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
//     fout.write(reinterpret_cast<char*>(&point), sizeof(point));
//     point.x = it->second.pick_pose.orientation.x;
//     point.y = it->second.pick_pose.orientation.y;
//     point.z = it->second.pick_pose.orientation.z;
//     point.w = it->second.pick_pose.orientation.w;
//     printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", point.x, point.y, point.z, point.w);
//     fout.write(reinterpret_cast<char*>(&point), sizeof(point));

//     point.x = it->second.place_pose.position.x;
//     point.y = it->second.place_pose.position.y;
//     point.z = it->second.place_pose.position.z;
//     printf("Position: x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
//     fout.write(reinterpret_cast<char*>(&point), sizeof(point));
//     point.x = it->second.place_pose.orientation.x;
//     point.y = it->second.place_pose.orientation.y;
//     point.z = it->second.place_pose.orientation.z;
//     point.w = it->second.place_pose.orientation.w;
//     printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", point.x, point.y, point.z, point.w);
//     fout.write(reinterpret_cast<char*>(&point), sizeof(point));
  
// }}

void PickPlace::SavePlaces(std::string filename) {
  
  std::ofstream outfile;
  outfile.open(filename);
  std::string header;
  header = "odom_combined";
  outfile << header;
  // std::cout<<header;
  outfile<<'\n';
  header = "r_wrist_roll_link";
  outfile<< header;
  outfile<<'\n';
  Point_t point;
  point.w = 1;
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) 
  {

    outfile<< it->first.c_str();
    outfile<<'\n';

      point.x = it->second.place_pose.position.x;
    outfile<< point.x;
    outfile<<",";
      point.y = it->second.place_pose.position.y;
      outfile<< point.y;
    outfile<<",";
      point.z = it->second.place_pose.position.z;
      outfile<< point.z;
    outfile<<",";
    outfile<<'\n';
    point.x = it->second.place_pose.orientation.x;
    outfile<< point.x;
    outfile<<",";
      point.y = it->second.place_pose.orientation.y;
    outfile<< point.y;
    outfile<<",";
      point.z = it->second.place_pose.orientation.z;
    outfile<<point.z;
    outfile<<",";
      point.w = it->second.place_pose.orientation.w;
    outfile<<point.w;
    outfile<<",";
    outfile<<'\n';

  }
}


void PickPlace::SaveCalibration(std::string filename) {
  
  std::ofstream outfile;
  outfile.open(filename);
  std::string header;
  header = "torso_lift_link";
  outfile << header;
  std::cout<<header;
  outfile<<'\n';
  header = "r_wrist_roll_link";
  outfile<< header;
  outfile<<'\n';
  Point_t point;
  point.w = 1;
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) 
  {

    outfile<< it->first.c_str();
    outfile<<'\n';
    point.x = it->second.pick_pose.position.x;
    outfile<< point.x;
    outfile<<",";
      point.y = it->second.pick_pose.position.y;
      outfile<< point.y;
    outfile<<",";
      point.z = it->second.pick_pose.position.z;
      outfile<< point.z;
    outfile<<",";
    outfile<<'\n';
    point.x = it->second.pick_pose.orientation.x;
    outfile<< point.x;
    outfile<<",";
      point.y = it->second.pick_pose.orientation.y;
      outfile<< point.y;
      outfile<<",";
      point.z = it->second.pick_pose.orientation.z;
      outfile<<point.z;
      outfile<<",";
      point.w = it->second.pick_pose.orientation.w;
      outfile<<point.w;
      outfile<<",";
      outfile<<'\n';

      point.x = it->second.place_pose.position.x;
    outfile<< point.x;
    outfile<<",";
      point.y = it->second.place_pose.position.y;
      outfile<< point.y;
    outfile<<",";
      point.z = it->second.place_pose.position.z;
      outfile<< point.z;
    outfile<<",";
    outfile<<'\n';
    point.x = it->second.place_pose.orientation.x;
    outfile<< point.x;
    outfile<<",";
      point.y = it->second.place_pose.orientation.y;
      outfile<< point.y;
      outfile<<",";
      point.z = it->second.place_pose.orientation.z;
      outfile<<point.z;
      outfile<<",";
      point.w = it->second.place_pose.orientation.w;


      outfile<<point.w;
      outfile<<",";
      outfile<<'\n';

  }
}


bool PickPlace::SendGoal(geometry_msgs::Pose goal) {
  static bool success = false;
  if (nh_.ok()) {
    bool finished_within_time = false;
    // ROS_INFO("Sending Goal");
    // finished_within_time = move_arm_.waitForResult(ros::Duration(45.0));
    // printf("\n  goal pos x %f  y %f  z %f \n", object_goal_map_["neutral"].pick_pose.position.x, object_goal_map_["neutral"].pick_pose.position.y, object_goal_map_["neutral"].pick_pose.position.z);
    // printf("\n  goal ori send goal x %f  y %f  z %f  w %f\n", object_goal_map_["neutral"].pick_pose.orientation.x, object_goal_map_["neutral"].pick_pose.orientation.y, object_goal_map_["neutral"].pick_pose.orientation.z, object_goal_map_["neutral"].pick_pose.orientation.z);
    arm_group_.setPoseTarget(goal);
    // ROS_INFO("     the target pose was succcessfully set to goal");

    // // attempt using move group tutorial
    // // set up planning scene interface and publisher for vizualizing plans in Rviz
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // // ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    // moveit_msgs::DisplayTrajectory display_trajectory;

    // // get info
    // ROS_INFO("Reference frame: %s", arm_group_.getPlanningFrame().c_str());
    // ROS_INFO("Reference frame: %s", arm_group_.getEndEffectorLink().c_str());

ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;


    // success = arm_group_.move();
  arm_group_.setNumPlanningAttempts(3);
  arm_group_.setPlanningTime(5.0);
  arm_group_.setGoalPositionTolerance(0.003); 
  arm_group_.setGoalOrientationTolerance(0.0075); 

    moveit::planning_interface::MoveGroup::Plan motion_plan;
    success = arm_group_.plan(motion_plan);

    // success = arm_group_.move();
    ROS_INFO("  Visualizing plan! %s",success?"":"FAILED");
    sleep(1.0);

    // if (!finished_within_time) {
    //   arm_group_.stop();
    //   ROS_INFO("Timed out achieving Goal");
    // } else {
      // robot_state::RobotStatePtr state = arm_group_.getCurrentState();
      // bool success = (state == moveit_msgs::MoveItErrorCodes::SUCCESS);
      if (success) {

        // now execute the plan

        // todo put back in!
        ROS_INFO("  Action will now execute!");
        arm_group_.execute(motion_plan);

        sleep(1.0);
        arm_group_.setStartStateToCurrentState();

        // ROS_INFO("Action finished: %s",state.toString().c_str());
        ROS_INFO("  Action finished:");
        return true;
      } else {
        // ROS_INFO("Action failed: %s",state.toString().c_str());
        ROS_INFO("  Action failed:");
        return false;
      }
    // }
  }
  return false;
} 


geometry_msgs::Pose PickPlace::GetArmPoseGoal() {
  // arm_navigation_msgs::MoveArmGoal goal;
  geometry_msgs::Pose goal;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // Get Current Arm Pose
  listener.waitForTransform("odom_combined", "r_wrist_roll_link",
     ros::Time(0), ros::Duration(3.0));
  listener.lookupTransform("odom_combined", "r_wrist_roll_link",
    ros::Time(0), transform);
  // goal.motion_plan_request.group_name = arm_.c_str();
  // goal.motion_plan_request.num_planning_attempts = 5;
  // goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  // arm_group_.group_name_ = arm_.c_str();
  arm_group_.setNumPlanningAttempts(15);
  arm_group_.setPlanningTime(15.0);

  // nh_.param<std::string>(
  //   "planner_id",
  //   // goal.motion_plan_request.planner_id,
  //   arm_group_.getDefaultPlannerId(arm_),
  //   std::string(""));
  // nh_.param<std::string>(
  //   "planner_service_name",
  //   // goal.planner_service_name,
  //   std::string("ompl_planning/plan_kinematic_path"));
  //ROS_INFO("              planner_id: %s", arm_group_.getDefaultPlannerId(arm_).c_str());
  ROS_INFO("HERE!!!!!");
  // // Setup position of Joint
  // goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  // goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
  // goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goal.position.x = transform.getOrigin().x();
  goal.position.y = transform.getOrigin().y();
  goal.position.z = transform.getOrigin().z();

  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;
  arm_group_.setGoalPositionTolerance(0.003); 

  // // Setup Orientation
  // goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goal.orientation.x = transform.getRotation().x();
  goal.orientation.y = transform.getRotation().y();
  goal.orientation.z = transform.getRotation().z();
  goal.orientation.w = transform.getRotation().w();
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  arm_group_.setGoalOrientationTolerance(0.0075);
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;

  return goal;
}


// TODO JB: ADDED FUNCTIONALITY TO CREATE SCENE OBJECTS
void PickPlace::SetSceneObjects() {

//----------------------------------------
// define objects 
sleep(1.0);
moveit_msgs::CollisionObject collision_object1;
moveit_msgs::CollisionObject collision_object2;
moveit_msgs::CollisionObject collision_object3;
collision_object1.header.frame_id = arm_group_.getPlanningFrame();
collision_object2.header.frame_id = arm_group_.getPlanningFrame();
collision_object3.header.frame_id = arm_group_.getPlanningFrame();

// -------
/* Define a bowl to add to the world. */
/* The id of the object is used to identify it. */
collision_object1.id = "Right_Bread";
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.1;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.1;
/* A pose for the bowl (specified relative to frame_id) */
geometry_msgs::Pose bowl_pose;
bowl_pose.orientation.w = 1.0;
bowl_pose.position.x =  0.55;
bowl_pose.position.y =  -0.35;
bowl_pose.position.z =  0.75; //objects_n3_v2.bin 
// bowl_pose.position.z =  0.3;

collision_object1.primitives.push_back(primitive);
collision_object1.primitive_poses.push_back(bowl_pose);
collision_object1.operation = collision_object1.ADD;

collision_objects_.push_back(collision_object1);

// -------
/* Define a cup to add to the world. */
/* The id of the object is used to identify it. */
collision_object2.id = "Left_Bread";
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.1;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.2;
geometry_msgs::Pose cup_pose;
cup_pose.orientation.w = 1.0;
cup_pose.position.x =  0.6;
cup_pose.position.y =  0.3;
cup_pose.position.z =  0.8; //objects_n3_v2.bin 
// cup_pose.position.z =  0.35;

collision_object2.primitives.push_back(primitive);
collision_object2.primitive_poses.push_back(cup_pose);
collision_object2.operation = collision_object2.ADD;

collision_objects_.push_back(collision_object2);

//----------------------------------------
// Add objects to world

// if collision_objects_.primitives.empty()
ROS_INFO("Add an object into the world");
planning_scene_interface_.addCollisionObjects(collision_objects_);

/* Sleep so we have time to see the object in RViz */
sleep(1.0);

}


int PickPlace::getIndex(std::string object) {
  // for index in vector collision_objects
  for ( int i = 0; i < collision_objects_.size(); i++) {
    // compare .id with object
    if ( collision_objects_[i].id == object) {
      // return index when match is found
      return i;
    }
  }
  return -1;
}

// TODO JB: ADDED FUNCTIONALITY TO CREATE TABLE
void PickPlace::SetSceneBounds() {

// define objects 
sleep(1.0);
moveit_msgs::CollisionObject collision_object1;
collision_object1.header.frame_id = arm_group_.getPlanningFrame();

// -------
/* Define a table to add to the world. */
/* The id of the object is used to identify it. */
collision_object1.id = "table";
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 1;
primitive.dimensions[1] = 2;

primitive.dimensions[2] = 0.85;

/* A pose for the table (specified relative to frame_id) */
geometry_msgs::Pose table_pose;
table_pose.orientation.w = 0;
table_pose.position.x =  0.86;
table_pose.position.y =  0;
table_pose.position.z =  0.3; //objects_n3_v2.bin 

collision_object1.primitives.push_back(primitive);
collision_object1.primitive_poses.push_back(table_pose);
collision_object1.operation = collision_object1.ADD;

collision_objects_.push_back(collision_object1);

//----------------------------------------
// Add objects to world

// if collision_objects_.primitives.empty()

ROS_INFO("Add an object into the world");
planning_scene_interface_.addCollisionObjects(collision_objects_);

/* Sleep so we have time to see the object in RViz */
sleep(1.0);
}



}  // namespace pr2
