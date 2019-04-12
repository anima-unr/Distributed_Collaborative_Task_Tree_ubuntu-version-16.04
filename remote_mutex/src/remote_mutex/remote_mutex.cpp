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
#include <string>
#include "remote_mutex/remote_mutex.h"
#include <robotics_task_tree_msgs/node_types.h>
#include "stdio.h"

namespace mutex {
RemoteMutex::RemoteMutex(std::string name, std::string topic) {
  msg.request.name = name;
  topic_ = topic;
}
RemoteMutex::RemoteMutex() {}
bool RemoteMutex::Lock(float potential) {
  msg.request.request = LOCK;
  msg.request.activation_potential = potential;
  if (ros::service::call(topic_.c_str(), msg)) {
    return msg.response.success;
  }
}
bool RemoteMutex::Release() {
  msg.request.request = RELEASE;
  // printf("\n\n\ttopic: %s \n\n", topic_.c_str());
  if (ros::service::call(topic_.c_str(), msg)) {
    return msg.response.success;
  }
}
}  // namespace mutex
