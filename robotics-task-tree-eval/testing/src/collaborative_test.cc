  /*
robotics-task-tree-eval
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include "robotics_task_tree_eval/behavior.h"
#include "robotics_task_tree_eval/node_types.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "collaborative_test");

  ros::Nodehandle nh("~");

  task_net::NodeId_t name_param;
  
  return 0;
}
