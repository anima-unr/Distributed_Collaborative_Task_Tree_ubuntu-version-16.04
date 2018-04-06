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

#include "collaborative_behavior.h"
namespace task_net {
CollabTestBehavior::CollabTestBehavior() {}
CollabTestBehavior::CollabTestBehavior(
    NodeId_t name,
    NodeList peers,
    NodeList children,
    NodeId_t parent,
    State_t state,
    std::string mutex_topic,
    std::string object,
    std::vector<float> pos,
    std::vector<float> neutral_pos,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : TableObject(
      name,
      peers,
      children,
      parent,
      state,
      mutex_topic,
      object,
      pos,
      neutral_pos,
      use_local_callback_queue,
      mtime) {
  state_.activation_potential = 1.0f;
}

CollabTestBehavior::~CollabTestBehavior() {}

bool CollabTestBehavior::Precondition() {
  return true;
}

bool CollabTestBehavior::ActivationPrecondition() {
  return mut.Lock(state_.activation_potential);
}

void CollabTestBehavior::Work() {
  boost::this_thread::sleep(boost::posix_time::millisec(5000));

  mut.Release();
}

bool CollabTestBehavior::CheckWork() {
  return true;
}

void CollabTestBehavior::UndoWork() {
  mut.Release();
}

void CollabTestBehavior::UpdateActivationPotential() {}
};  // namespace task_net
