/*
remote_mutex
Copyright (C) 2015  Luke Fraser

remote_mutex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

remote_mutex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with remote_mutex.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <string>
#include <vector>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "remote_mutex/remote_mutex.h"
#include "timeseries_recording_toolkit/record_timeseries_data_to_file.h"


class RemoteMutexService;

void Record(RemoteMutexService* mut);

task_net::NodeBitmask GetBitmask(std::string name) {
    // ROS_INFO("Node::GetBitmask was called!!!!\n");
  // Split underscores
  std::vector<std::string> split_vec;
  boost::algorithm::split(split_vec, name,
    boost::algorithm::is_any_of("_"));
  task_net::NodeBitmask mask;
  // node_type
  mask.type  = static_cast<uint8_t>(atoi(split_vec[1].c_str()));
  mask.robot = static_cast<uint8_t>(atoi(split_vec[2].c_str()));
  mask.node  = static_cast<uint16_t>(atoi(split_vec[3].c_str()));
  return mask;
}


class RemoteMutexService {
 public:
  bool locked;
  std::string owner;
  ros::ServiceServer service;
  ros::NodeHandle ns;
  ros::Subscriber state_subscriber_;
  std::string root_topic_;
  float activation_potential;
  boost::mutex mut;
  boost::thread* record_thread;
  std::ofstream file;
  recording_toolkit::FilePrintRecorder record_object;
  int enum_robot_; 

  // ros info
  robotics_task_tree_msgs::State top_level_state_;

  explicit RemoteMutexService(const char* name)
      : record_object("/home/janelle/onr_ws/src/Distributed_Collaborative_Task_Tree/Data/remote_mutex.csv",
        100) {
    locked = false;
    owner = "";
    activation_potential = 0.0f;
    service = ns.advertiseService(
      name,
      &RemoteMutexService::MutexRequest,
      this);

    ns.param<std::string>( "/topic", root_topic_, "AND_2_0_006_state");
    ns.param<int>( "/enum_robot", enum_robot_, 0);
    state_subscriber_ = ns.subscribe(root_topic_, 1000, &RemoteMutexService::RootStateCallback, this );

    record_thread = new boost::thread(&Record, this);
    record_object.StartRecord();
  }

  ~RemoteMutexService() {
    delete record_thread;
    // record_object.WaitUntilFinishedWriting();
    record_object.StopRecord();
  }

  void RecordToFile() {
    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - time_t_epoch;
    double seconds = (double)diff.total_seconds() + (double)diff.fractional_seconds() / 1000000.0;
    record_object.RecordPrintf("%f, %s\n", seconds, owner.c_str() );
  }

  void RootStateCallback( robotics_task_tree_msgs::State msg)
  {
    ROS_DEBUG( "RootStateCalback");
    // get the current state variable and save it (for MutexRequest to read later)
    top_level_state_ = msg;
  }

  bool MutexRequest(remote_mutex::remote_mutex_msg::Request &req,
      remote_mutex::remote_mutex_msg::Response &res) {

    boost::this_thread::sleep(boost::posix_time::millisec(200));
    if (req.request) {

      // sleep diff rates between pr2 and baxter to offset them activating nodes at the same time...
      int offset = 75*enum_robot_;
      boost::this_thread::sleep(boost::posix_time::millisec(200+offset));
      ROS_DEBUG("\tWAITED %d", 200+offset);


      ROS_INFO("asking for mutex lock [%f / %f] %s", activation_potential, req.activation_potential, req.name.c_str());

      if (locked) {
        res.success = false;
        ROS_DEBUG("Mutex Already Locked - Denied Access: %s", req.name.c_str());
      } 
      else {

        // TODO: Remove this line!
        ROS_INFO("top_level_state_.highest.node: %d", top_level_state_.highest.node);

        // is this node the node that has the higest activation potential
        if( is_eq(GetBitmask(req.name), top_level_state_.highest) )
        {
          mut.lock();
          activation_potential = req.activation_potential;
          mut.unlock();
          if( activation_potential > 0.001 )
          {
            ROS_INFO("Mutex Locked - Granted Access: %s", req.name.c_str());
            mut.lock();
            locked = true;
            owner = req.name;
            mut.unlock();
            res.success = true;        
          }
          else
          {
            ROS_INFO( "Activation Potential <= 0, no lock granted");
            res.success = false;
          }
        }
        else {
          ROS_INFO("Not Highest Activation Potential - Denied Access: [%d %d %d/ %s]", top_level_state_.highest.type, top_level_state_.highest.robot, top_level_state_.highest.node, req.name.c_str());
          res.success = false;
        }
      }
    } 
    else 
    {
      ROS_INFO( "asking for mutex release: [%s / %s]", owner.c_str(), req.name.c_str());
      if (locked) 
      {
        if (req.name == owner) 
        {
          mut.lock();
          locked = false;
          owner = "";
          activation_potential = 0.0f;
          mut.unlock();
          res.success = true;
          ROS_INFO("Mutex Unlocked - Granted Access: %s", req.name.c_str());
        } 
        else 
        {
          res.success = false;
          ROS_INFO("Mutex Locked - Denied Access: %s", req.name.c_str());
        }
      }
      else 
      {
        res.success = false;
        ROS_INFO("Mutex Already Unlocked: %s", req.name.c_str());
      }
    }

    return true;
  }

};

void Record(RemoteMutexService* mut) {
  while (true) {
    mut->RecordToFile();
    boost::this_thread::sleep(boost::posix_time::millisec(50));
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "remote_mutex_server");
  if (argc >= 2) {
    RemoteMutexService mutex(argv[1]);
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::waitForShutdown();
  } else {
    ROS_FATAL("A Mutex Name is a required Parameter - None Given");
    return -1;
  }
  return 0;
}
