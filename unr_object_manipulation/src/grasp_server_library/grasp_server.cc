/*Copyright 2016 Luke Fraser

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
#include "grasp_server_library/grasp_server.h"
#include "log.h"
#include <iostream>

namespace grasplib {
////////////////////////////////////////////////////////////////////////////////
// Pose
////////////////////////////////////////////////////////////////////////////////
Pose::Pose() {}
Pose::~Pose() {}

bool Pose::SetPose(geometry_msgs::Pose pose) {
  pose_ = pose;
  return true;
}
geometry_msgs::Pose Pose::GetPose() const {
  return pose_;
}
Eigen::Vector4f Pose::GetPoseNormal(geometry_msgs::Pose pose) const {}
Eigen::Vector4f Pose::GetPoseNormal() const {}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Grasp
////////////////////////////////////////////////////////////////////////////////
Grasp::Grasp() {}
Grasp::~Grasp() {}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// ObjectPickPlace
////////////////////////////////////////////////////////////////////////////////
ObjectPickPlace::ObjectPickPlace() {}
ObjectPickPlace::~ObjectPickPlace() {}

bool ObjectPickPlace::GenerateGraspFromPose() {
  return false;
}
bool ObjectPickPlace::GetPick() {
  return false;
}
bool ObjectPickPlace::GetPlace() {
  return false;
}
bool ObjectPickPlace::SetPick() {
  return false;
}
bool ObjectPickPlace::SetPlace() {
  return false;
}
uint32_t ObjectPickPlace::GetType() {
  return type_;
}
bool ObjectPickPlace::SetType() {
  return false;
}
std::string ObjectPickPlace::GetName() {
  return name_;
}
bool ObjectPickPlace::SetName() {
  return false;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// GraspServer
////////////////////////////////////////////////////////////////////////////////
GraspServer::GraspServer(std::string arm) {}
GraspServer::~GraspServer() {}

bool GraspServer::AddObject(std::string object) {
  if (objects_.count(object) == 0) {
    objects_[object];
    return true;
  }
  return false;
}
bool GraspServer::RemoveObject(std::string object) {
  if (objects_.count(object) > 0) {
    objects_.erase(object);
    return true;
  }
  return false;
}
bool GraspServer::LoadObjects(std::string filename) {
  LOG_INFO("[WARNING]: File [%s] - Load not implemented!", filename.c_str());
  return false;
}
bool GraspServer::LoadObjects(std::vector<ObjectPickPlace> objects) {
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    objects_[it->GetName()] =  *it;
  }
  return true;
}
bool GraspServer::SaveObjects(std::string filename) {
  LOG_INFO("[WARNING]: File [%s] - Save not implemented!", filename.c_str());
  return false;
}
bool GraspServer::SaveObjects(
    std::string filename, std::vector<ObjectPickPlace> objects) {
  LOG_INFO("[WARNING]: File [%s] - Save not implemented!", filename.c_str());
  return false;
}
bool GraspServer::MergeObjects(std::string filename) {
  LOG_INFO("[WARNING]: File [%s] - Merge not implemented!", filename.c_str());
  return false;
}
bool GraspServer::MergeObjects(
    std::string filename, std::vector<ObjectPickPlace> objects) {
  LOG_INFO("[WARNING]: File [%s] - Merge not implemented!", filename.c_str());
  return false;
}
Pose GraspServer::GetArmPose(std::string arm) {
  Pose pose;
  geometry_msgs::Pose msg;
  msg.position.x = 10.0;
  pose.SetPose(msg);
  return pose;
}
Grasp GraspServer::GenerateGraspFromPose(Pose pose) {
  LOG_INFO("[WARNING]: Grasp Generation not implemented!");
  return Grasp();
}
ObjectPickPlace GraspServer::GetObject(std::string object) {
  return ObjectPickPlace();
}
std::vector<ObjectPickPlace> GraspServer::GetObjects(
    std::vector<std::string> objects) {
  std::vector<ObjectPickPlace> result;
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    if (objects_.count(*it) > 0)
      result.push_back(objects_[*it]);
  }
  return result;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}  // namespace grasplib
