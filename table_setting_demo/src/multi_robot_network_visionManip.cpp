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

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <signal.h>
#include <vector>
#include <string>
#include <map>
#include "robotics_task_tree_eval/behavior.h"
#include "robotics_task_tree_msgs/node_types.h"
#include "table_setting_demo/table_object_behavior_VisionManip.h"


typedef std::vector<std::string> NodeParam;
// enum ROBOT {
//   PR2=0,
//   BAXTER=1
// } ;


void EndingFunc(int signal) {
  printf("Closing Program...\n");
  ros::shutdown();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "behavior_network", ros::init_options::NoSigintHandler);
  signal(SIGINT, EndingFunc);
  ros::NodeHandle nh_("~");

  //current state error 
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  task_net::Node ** network;

  task_net::NodeId_t name_param;
  std::vector<std::string> peers_param_str;
  task_net::NodeList peers_param;
  std::vector<std::string> children_param_str;
  task_net::NodeList children_param;
  task_net::NodeId_t parent_param;
  NodeParam nodes;
  std::string object;
  std::string obj_name;
  std::vector<float> neutral_object_pos;
  std::vector<float> object_pos;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  // get the robot
  std::string Robot;
  nh_.getParam("robot", Robot);
  ROBOT robot_des;
  if(Robot == "PR2") {
    robot_des = PR2;
  }
  else {
    robot_des = BAXTER;
  }

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
  std::string param_ext_peers = "peers";
  for(int i=0; i < nodes.size(); ++i) {

    ROS_WARN("HERE;");

    // Get name
    name_param.topic = nodes[i];
    // printf("name: %s\n", name_param.topic.c_str());

    // only init the nodes for the correct robot!!!
    int robot;
    if (nh_.getParam((param_prefix + nodes[i] + "/mask/robot").c_str(), robot)) {
      if(robot == robot_des) {

        printf("Creating Task Node for:\n");
        printf("\tname: %s\n", name_param.topic.c_str());
        // get parent
        if (nh_.getParam((param_prefix + nodes[i]
            + "/" + param_ext_parent).c_str(), parent_param.topic)) {
          printf("Node: %s Parent: %s\n", nodes[i].c_str(), parent_param.topic.c_str());
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
        // get peers
        peers_param.clear();
        if (nh_.getParam((param_prefix + nodes[i]
              + "/" + param_ext_peers).c_str(), peers_param_str)) {
            for (int j = 0; j < peers_param_str.size(); ++j) {
              task_net::NodeId_t temp;
              temp.topic = peers_param_str[j];
              temp.pub = NULL;
              peers_param.push_back(temp);
              printf("Node: %s Peer: %s\n", nodes[i].c_str(), temp.topic.c_str());
            }
          }

        // Create Node
        task_net::State_t state;
        task_net::Node * test;

        int type;
        if (nh_.getParam((param_prefix + nodes[i] + "/mask/type").c_str(), type)) {
          // printf("Node: %s NodeType: %d\n", nodes[i].c_str(), type);
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
          case task_net::BEHAVIOR_VM:
           ROS_INFO("Children Size: %lu", children_param.size());
            object = name_param.topic.c_str();
           // get the name of the object of corresponding node:
            nh_.getParam((param_prefix + nodes[i] + "/object").c_str(), obj_name);
            // set up network for corresponding node:
            ros::param::get(("/ObjectPositions/"+obj_name).c_str(), object_pos);
            //ROS_INFO("Found %s at loc %f, %f, %f", obj_name.c_str(), object_pos[0], object_pos[1], object_pos[2]);
            network[i] = new task_net::TableObject_VisionManip(name_param,
                                      peers_param,
                                      children_param,
                                      parent_param,
                                      state,
                                      obj_name,
                                      "/right_arm_mutex",
                                      object_pos,
                                      false);
            // network[i] = new task_net::TableObject_VisionManip();
            // network[i] = new task_net::DummyBehavior(name_param,
            //                             peers_param,
            //                             children_param,
            //                             parent_param,
            //                             state,
            //                             false);
            // printf("\ttask_net::PLACE %d\n",task_net::PLACE);
            printf("/ObjectPositions/%s\n\n",obj_name.c_str());
            break;
          case task_net::ROOT:
          default:
            network[i] = NULL;
            // printf("\ttask_net::ROOT %d\n",task_net::ROOT);
            break;
        } //switch
        if( network[i] != NULL )
        {
          network[i]->init();
        }
      }
      printf("MADE 5\n");
    }
  } // for
  printf("MADE 6 - now spinning\n");
  ros::spin();
  return 0;
}
