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
#ifndef TESTING_INCLUDE_COLLABORATIVE_TEST_H_
#define TESTING_INCLUDE_COLLABORATIVE_TEST_H_
#include "table_setting_demo/table_object_behavior.h"
namespace task_net {
class CollabTestBehavior : public TableObject {
 public:
  CollabTestBehavior();
  CollabTestBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string mutex_topic,
    std::string object,
    std::vector<float> pos,
    std::vector<float> neutral_pos,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(1000));
  virtual ~CollabTestBehavior();
  // virtual void UpdateActivationPotential();
 protected:
  virtual bool Precondition();
  virtual bool ActivationPrecondition();
  virtual void Work();
  virtual bool CheckWork();
  virtual void UndoWork();
  virtual void UpdateActivationPotential();
};
};  // namespace task_net
#endif  // TESTING_INCLUDE_COLLABORATIVE_TEST_H_
