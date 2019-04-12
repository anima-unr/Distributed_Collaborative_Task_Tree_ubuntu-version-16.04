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
// Testing Application API Object
#include "grasp_server_library/grasp_server.h"
#include <gtest/gtest.h>
#include "ros/ros.h"

TEST(SimpleTests, AddObject) {
  grasplib::GraspServer server("right");
  ASSERT_EQ(server.AddObject("fork"), true);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Test_Node");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
