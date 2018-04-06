/*
table_setting_demo
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
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <boost/thread/thread.hpp>
#include <boost/assert.hpp>
#include "robotics_task_tree_eval/behavior.h"
#include "table_setting_demo/table_object_behavior.h"
#include "collaborative_behavior.h"
#include "log.h"

class CollaborativeTest {
 public:
  ros::NodeHandle nh_;
  std::vector< boost::shared_ptr<task_net::Node> > net_robot_1;
  std::vector< boost::shared_ptr<task_net::Node> > net_robot_2;
  std::vector<std::string> nodes;

  CollaborativeTest() : nh_("~") {
    // Get parameters for test
    if (!nh_.getParam("NodeList", nodes)) {
      LOG_INFO("Error: Node Parameter NOTFound!");
      BOOST_ASSERT(false);
    }

    // Allocate node shared pointer array
    boost::shared_ptr< boost::shared_ptr< task_net::Node > > net(
      new boost::shared_ptr<task_net::Node>[nodes.size()],
      std::default_delete< boost::shared_ptr<task_net::Node>[] >()
    );

    std::string param_prefix = "Nodes/";
    std::string param_ext_children = "children";
    std::string param_ext_parent = "parent";
    std::string param_ext_peers = "peers";

    std::vector<std::string> peers_param_str;
    std::vector<std::string> children_param_str;

    task_net::NodeId_t name_param;
    task_net::NodeList peers_param;
    task_net::NodeList children_param;
    task_net::NodeId_t parent_param;
    task_net::State state;
    std::string object;
    std::vector<float> neutral_object_pos;
    std::vector<float> object_pos;


    BOOST_ASSERT(nh_.getParam("ObjectPositions/neutral", neutral_object_pos));
    for (std::vector<std::string>::iterator it = nodes.begin();
        it != nodes.end(); ++it) {
      std::cout << "NODE: " << *it << std::endl;

      name_param.topic = *it;

      // Get Parent
      BOOST_ASSERT(
        nh_.getParam(
          (param_prefix + *it + "/" + param_ext_parent).c_str(),
          parent_param.topic
        )
      );

      std::cout << "PARENT: " << parent_param.topic << std::endl;

      // Get the child parameters
      children_param.clear();
      children_param_str.clear();
      BOOST_ASSERT(
        nh_.getParam(
          (param_prefix + *it + "/" + param_ext_children).c_str(),
          children_param_str
        )
      );
      if (children_param_str.size() > 0 && children_param_str[0] != "NONE") {
        for (std::vector<std::string>::iterator jt = children_param_str.begin();
            jt != children_param_str.end(); ++jt) {
          // std::cout << *jt << std::endl;
          task_net::NodeId_t temp;
          temp.topic = *jt;
          temp.pub = NULL;
          children_param.push_back(temp);
        }
      }
      for (int i = 0; i < children_param.size(); ++i) {
        std::cout << "Children:[" << i << "] " << children_param[i].topic << std::endl;
      }

      // Get the Peers parameters
      peers_param_str.clear();
      peers_param.clear();
      BOOST_ASSERT(
        nh_.getParam(
          (param_prefix + *it + "/" + param_ext_peers).c_str(),
          peers_param_str
        )
      );
      if (peers_param_str.size() > 0 && peers_param_str[0] != "NONE") {
        for (std::vector<std::string>::iterator jt = peers_param_str.begin();
            jt != peers_param_str.end(); ++jt) {
          task_net::NodeId_t temp;
          temp.topic = *jt;
          temp.pub = NULL;
          peers_param.push_back(temp);
          std::cout << "PEERS: " << *jt << std::endl;
        }
      }

      // Create the nodes with the appropriate attributes
      int type;
      BOOST_ASSERT(
        nh_.getParam((param_prefix + *it + "/mask/type").c_str(), type)
      );
      int robot;
      BOOST_ASSERT(
        nh_.getParam((param_prefix + *it + "/mask/robot").c_str(), robot)
      );

      boost::shared_ptr<task_net::Node> node;
      switch (type) {
        case task_net::THEN:
          node = boost::shared_ptr<task_net::Node>(
              new task_net::ThenBehavior(name_param,
                peers_param,
                children_param,
                parent_param,
                state,
                false));
          robot == task_net::PR2 ? 
            net_robot_1.push_back(node) : net_robot_2.push_back(node);
          break;
        case task_net::OR:
          node = boost::shared_ptr<task_net::Node>(
              new task_net::OrBehavior(name_param,
                peers_param,
                children_param,
                parent_param,
                state,
                false));
          robot == task_net::PR2 ? 
            net_robot_1.push_back(node) : net_robot_2.push_back(node);
          break;
        case task_net::AND:
          node = boost::shared_ptr<task_net::Node>(
              new task_net::AndBehavior(name_param,
                peers_param,
                children_param,
                parent_param,
                state,
                false));
          robot == task_net::PR2 ? 
            net_robot_1.push_back(node) : net_robot_2.push_back(node);
          break;
        case task_net::BEHAVIOR:
          BOOST_ASSERT_MSG(
            nh_.getParam(("ObjectPositions/" + *it).c_str(), object_pos),
            ("ObjectPositions/" + *it).c_str()
          );
          node = boost::shared_ptr<task_net::Node>(
              new task_net::CollabTestBehavior(name_param,
                peers_param,
                children_param,
                parent_param,
                state,
                robot == task_net::PR2 ? "/right_arm_mutex" : "/left_arm_mutex",
                *it,
                neutral_object_pos,
                object_pos));
          robot == task_net::PR2 ? 
            net_robot_1.push_back(node) : net_robot_2.push_back(node);
          break;
        case task_net::ROOT:
          break;
        default:
          LOG_INFO("Error Unknown Node type!");
          break;
      }
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "collaborative_test");

  CollaborativeTest test;
  ros::spin();
  return 0;
}
