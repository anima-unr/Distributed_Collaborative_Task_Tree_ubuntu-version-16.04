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
#ifndef REMOTE_MUTEX_H_
#define REMOTE_MUTEX_H_
#include <string>
#include "ros/ros.h"
#include "remote_mutex/remote_mutex_msg.h"
#include <robotics_task_tree_msgs/node_types.h>

#define LOCK true
#define RELEASE false

namespace mutex {
class RemoteMutex {
 public:
  RemoteMutex(std::string name, std::string topic);
  RemoteMutex();
  bool Lock(float potential);
  bool Release();

 private:
  remote_mutex::remote_mutex_msg msg;
  std::string topic_;
};
}  // namespace mutex
#endif // REMOTE_MUTEX_H_
