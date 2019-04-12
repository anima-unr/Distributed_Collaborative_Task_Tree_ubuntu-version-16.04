/*
Copyright 2016 Luke Fraser

This file is part of UNR_Object_Manipulation.

UNR_Object_Manipulation is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

UNR_Object_Manipulation is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with UNR_Object_Manipulation.  If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef INCLUDE_GRASP_SERVER_LIBRARY_GRASP_SERVER_H_
#define INCLUDE_GRASP_SERVER_LIBRARY_GRASP_SERVER_H_
#include <stdint.h>
#include <Eigen/Eigen>
#include <string>
#include <unordered_map>
#include <vector>
#include "moveit_msgs/Grasp.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "unr_object_manipulation/uomconfig.h"
#include "ros/ros.h"

namespace grasplib {
enum OBJECT_TYPE_e {
  DYNAMIC,
  STATIC
} OBJECT_TYPE;

class Pose {
 public:
  Pose();
  ~Pose();

  bool SetPose(geometry_msgs::Pose pose);
  geometry_msgs::Pose GetPose() const;
  Eigen::Vector4f GetPoseNormal(geometry_msgs::Pose pose) const;
  Eigen::Vector4f GetPoseNormal() const;

 private:
  geometry_msgs::Pose pose_;
};

class Grasp {
 public:
  Grasp();
  ~Grasp();
 private:
  moveit_msgs::Grasp grasp_;
};
////////////////////////////////////////////////////////////////////////////////
// Class:
//   ObjectPickPlace
// Description:
//   This class stores all grasp and object information needed to produce a full
//   pick and place of the object. It will store the pick grasp as well as the
//   place grasp.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class ObjectPickPlace {
 public:
  ObjectPickPlace();
  ~ObjectPickPlace();

  bool GenerateGraspFromPose();
  bool GetPick();
  bool GetPlace();
  bool SetPick();
  bool SetPlace();
  uint32_t GetType();
  bool SetType();
  std::string GetName();
  bool SetName();

 private:
  std::string name_;
  uint32_t type_;
  Grasp pick_;
  Grasp place_;
  uint32_t status_;
};

class GraspServer {
 public:
  explicit GraspServer(std::string arm);
  ~GraspServer();

  // Public Facing API functions

  //////////////////////////////////////////////////////////////////////////////
  // Status Functions
  //////////////////////////////////////////////////////////////////////////////
  bool CheckServerState(std::string *status);

  //////////////////////////////////////////////////////////////////////////////
  // Grasp Functions
  //////////////////////////////////////////////////////////////////////////////
  bool AddObject(std::string object);
  bool RemoveObject(std::string object);
  bool LoadObjects(std::string filename);
  bool LoadObjects(std::vector<ObjectPickPlace> objects);
  bool SaveObjects(std::string filename);
  bool SaveObjects(std::string filename, std::vector<ObjectPickPlace> objects);
  bool MergeObjects(std::string filename);
  bool MergeObjects(std::string filename, std::vector<ObjectPickPlace> objects);
  Pose GetArmPose(std::string arm);
  Grasp GenerateGraspFromPose(Pose pose);
  ObjectPickPlace GetObject(std::string object);
  std::vector<ObjectPickPlace> GetObjects(std::vector<std::string> objects);

  //////////////////////////////////////////////////////////////////////////////
  // Testing Functions
  //////////////////////////////////////////////////////////////////////////////
  bool SendPickPlaceGoal(std::string object);

  //////////////////////////////////////////////////////////////////////////////
  // PARAMETERS
  //////////////////////////////////////////////////////////////////////////////
  bool PostParameters();

  //////////////////////////////////////////////////////////////////////////////
  // CALLBACK FUNCTIONS
  //////////////////////////////////////////////////////////////////////////////
  bool _GenerateGraspCB(std_msgs::String req, Grasp res);
  bool _GeneratePoseCB(std_msgs::String req, Pose res);

 protected:
  ros::NodeHandle nh_;
  std::unordered_map<std::string, ObjectPickPlace> objects_;
};
}  // namespace grasplib
#endif  // INCLUDE_GRASP_SERVER_LIBRARY_GRASP_SERVER_H_
