/*
baxter_demos
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
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <signal.h>
#include <vector>
#include <string>
#include <map>
#include <vector>
#include "robotics_task_tree_eval/behavior.h"
#include "robotics_task_tree_msgs/node_types.h"
#include "table_setting_demo/table_object_behavior.h"

typedef std::vector<std::string> NodeParam;

class TableSetting {
 public:
  ros::NodeHandle nh_;
  task_net::Node ** network;
  TableSetting() : nh_("~") {
    // import Nodes and parameters
    task_net::NodeId_t name_param;
    std::vector<std::string> peers_param_str;
    task_net::NodeList peers_param;
    std::vector<std::string> children_param_str;
    task_net::NodeList children_param;
    task_net::NodeId_t parent_param;
    NodeParam nodes;
    task_net::State state;
    std::string object;
    std::string obj_name;
    std::vector<float> neutral_object_pos;
    std::vector<float> object_pos;

    ros::param::get("/ObjectPositions/neutral", neutral_object_pos);
    if( neutral_object_pos.size() > 0 )
      printf("neutral: %f\n", neutral_object_pos[0]); //this is getting stuff correctly - via the pr2 pick service stuff

    if (nh_.getParam("NodeList", nodes)) {
      printf("Tree Size: %lu\n", nodes.size());
    }
    else { printf("No nodeList params!!!\n");
    }
    network = new task_net::Node*[nodes.size()];

    // Grab Node Attributes
    std::string param_prefix = "Nodes/";
    std::string param_ext_children = "children";
    std::string param_ext_parent = "parent";
    for(int i=0; i < nodes.size(); ++i) {
      // Get name
      name_param.topic = nodes[i];
      // get parent
      if (nh_.getParam((param_prefix + nodes[i]
          + "/" + param_ext_parent).c_str(), parent_param.topic)) {
        printf("Node: %s Parent: %s\n", nodes[i].c_str(),
          parent_param.topic.c_str());
      }
      // get children
      children_param.clear();
      if (nh_.getParam((param_prefix + nodes[i]
          + "/" + param_ext_children).c_str(), children_param_str)) {
        for (int j = 0; j < children_param_str.size(); ++j) {
          task_net::NodeId_t temp;
          temp.topic = children_param_str[j];
          temp.pub = NULL;
          children_param.push_back(temp);
          printf("Node: %s Child: %s\n", nodes[i].c_str(), temp.topic.c_str());
        }
      }

      // Create Node
      int type;
      if (nh_.getParam((param_prefix + nodes[i] + "/mask/type").c_str(), type)) {
        printf("Node: %s NodeType: %d\n", nodes[i].c_str(), type);
      }

      switch (type) {
        case task_net::THEN:
          network[i] = new task_net::ThenBehavior(name_param,
                                      peers_param,
                                      children_param,
                                      parent_param,
                                      state,
                                      "N/A",
                                      false);
          // printf("\ttask_net::THEN %d\n",task_net::THEN);
          break;
        case task_net::OR:
          network[i] = new task_net::OrBehavior(name_param,
                                      peers_param,
                                      children_param,
                                      parent_param,
                                      state,
                                      "N/A",
                                      false);
          // printf("\ttask_net::OR %d\n",task_net::OR);
          break;
        case task_net::AND:
          network[i] = new task_net::AndBehavior(name_param,
                                      peers_param,
                                      children_param,
                                      parent_param,
                                      state,
                                      "N/A",
                                      false);
          // printf("\ttask_net::AND %d\n",task_net::AND);
          break;
        case task_net::BEHAVIOR:
          ROS_INFO("Children Size: %lu", children_param.size());
          object = name_param.topic.c_str();


          // get the name of the object of corresponding node:
          nh_.getParam((param_prefix + nodes[i] + "/object").c_str(), obj_name);
          // printf("%s\n\n\n",obj_name.c_str());
          // printf("%s\n\n\n\n",("/ObjectPositions/"+obj_name).c_str());

          // set up network for corresponding node:
          ros::param::get(("/ObjectPositions/"+obj_name).c_str(), object_pos);
          network[i] = new task_net::TableObject(name_param,
                                    peers_param,
                                    children_param,
                                    parent_param,
                                    state,
                                    obj_name.c_str(),
                                    "/right_arm_mutex",
                                    object_pos,
                                    neutral_object_pos,
                                    false);

          // if (object =="PLACE_3_0_010") {
          //   ros::param::get("/ObjectPositions/cup", object_pos);
          //   network[i] = new task_net::TableObject(name_param,
          //                           peers_param,
          //                           children_param,
          //                           parent_param,
          //                           state,
          //                           "/right_arm_mutex",
          //                           "cup",
          //                           object_pos,
          //                           neutral_object_pos,
          //                           false);

          // } else if (object =="PLACE_3_0_012") {
          //   ros::param::get("/ObjectPositions/plate", object_pos);
          //   network[i] = new task_net::TableObject(name_param,
          //                           peers_param,
          //                           children_param,
          //                           parent_param,
          //                           state,
          //                           "/right_arm_mutex",
          //                           "plate",
          //                           object_pos,
          //                           neutral_object_pos,
          //                           false);
          // } else if (object =="PLACE_3_0_013") {
          //   ros::param::get("/ObjectPositions/bowl", object_pos);
          //   network[i] = new task_net::TableObject(name_param,
          //                           peers_param,
          //                           children_param,
          //                           parent_param,
          //                           state,
          //                           "/right_arm_mutex",
          //                           "bowl",
          //                           object_pos,
          //                           neutral_object_pos,
          //                           false);
          // } else {
          //   printf("ERROR - wrong node for tree: %s\n", name_param.topic.c_str());
          // }
          // printf("\ttask_net::BEHAVIOR %d\n",task_net::BEHAVIOR);
          break;
        case task_net::ROOT:
        default:
          network[i] = NULL;
          // printf("\ttask_net::ROOT %d\n",task_net::ROOT);
          break;
      }

    }
    // Initialzie Nodes

  }
  ~TableSetting() {

  }

};

void EndingFunc(int signal) {
  printf("Closing Program...\n");
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_network", ros::init_options::NoSigintHandler);
  signal(SIGINT, EndingFunc);

  printf("This is happening!!!!!!\n");
  TableSetting SetTable;
  ROS_INFO("\n\nThis is finished!!!!!!\n");
  ros::spin();
  return 0;
}