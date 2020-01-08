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
#ifndef INCLUDE_TABLE_SETTING_TABLE_OBJECT_H_
#define INCLUDE_TABLE_SETTING_TABLE_OBJECT_H_
#include <string>
#include "robotics_task_tree_eval/behavior.h"
#include "robotics_task_tree_msgs/node_types.h"
#include "remote_mutex/remote_mutex.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace task_net { 

class TableObject : public Behavior {
 public:
  TableObject();
  TableObject(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    std::string mutex_topic,
    std::vector<float> pos,
    std::vector<float> neutral_pos,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(1000)
    );
  virtual ~TableObject();
  virtual void UpdateActivationPotential();
 protected:
  virtual void PickAndPlace(std::string object);
  virtual bool Precondition();
  virtual bool ActivationPrecondition();
  virtual void Work();
  virtual bool PickAndPlaceDone();
  virtual bool CheckWork();
  virtual void UndoWork();

 protected:
  mutex::RemoteMutex mut;

  std::string object_;
  std::string object_id_;
  std::vector<float> object_pos;
  std::vector<float> neutral_object_pos;
  bool dynamic_object;

  // frame info
  std::string root_frame_;
  std::string manip_frame_;
  
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  
  // debugging info
  bool ready_to_publish_ = false;
  ros::Publisher marker_pub_;

  // kludge!!
  bool first_time_;

};
}  // namespace task_net
#endif  // INCLUDE_TABLE_SETTING_TABLE_OBJECT_H_
