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
#include <stdio.h>
#include "ros/ros.h"
#include "unr_object_manipulation/object_request.h"
// #include "grasp_commander/grasp_commander.h"

namespace uom = unr_object_manipulation;

class GraspCommanderService {
 public:
  GraspCommanderService() {}
  ~GraspCommanderService() {}
  bool ObjectPickPlaceRequest(
  uom::object_request::Request &req,
  uom::object_request::Response &res) {
    printf("Object Pick Place Request\n");
    return true;
  }
 private:
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_commander_service");

  GraspCommanderService obj;

  ros::NodeHandle nh;
  // Attach Service callbacks
  nh.advertiseService("object_pick_place_service",
    &GraspCommanderService::ObjectPickPlaceRequest, &obj);

  return 0;
}
