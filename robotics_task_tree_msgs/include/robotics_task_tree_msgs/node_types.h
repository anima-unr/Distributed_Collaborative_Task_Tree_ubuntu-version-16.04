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
#ifndef NODE_TYPES_H_
#define NODE_TYPES_H_

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <assert.h>
#include <stdint.h>
#include <string>
#include <vector>
#include "robotics_task_tree_msgs/State.h"
#include "robotics_task_tree_msgs/ControlMessage.h"

namespace task_net {

typedef enum {  // Eight possible node types
  THEN = 0,     // 0
  OR,           // 1
  AND,          // 2
  BEHAVIOR,     // 3
  ROOT,         // 4
  PLACE,        // 5
  BEHAVIOR_VM,  // 6
} NodeTypes_t;

typedef enum {  // Eight possible robots
  PR2 = 0, // 0
  BAXTER,  // 1
} RobotTypes;

struct NodeBitmask {
  uint8_t type;
  uint8_t robot;
  uint16_t node;
};

struct BitmaskLessThan {
  bool operator()(const NodeBitmask &l, const NodeBitmask &r) {
    return *reinterpret_cast<const uint32_t*>(&l) <
      *reinterpret_cast<const uint32_t*>(&r);
  }
};

struct State {
  NodeBitmask owner;  // If owner is null node is inactive
  bool active;
  bool done;
  bool check_peer;
  bool peer_okay;
  float activation_level;
  float activation_potential;
  bool peer_active;
  bool peer_done;
  NodeBitmask highest;
  float highest_potential;
  int parent_type;
  float suitability;
};
typedef State State_t;

struct NodeId {
  std::string topic;
  NodeBitmask mask;
  ros::Publisher * pub;
  State_t state;
};
typedef NodeId NodeId_t;


struct ControlMessage {
  NodeBitmask sender;
  int type;
  float activation_level;
  float activation_potential;
  bool done;
  bool active;
  NodeBitmask highest;
  int parent_type;
};

typedef std::vector<NodeId_t> NodeList;
typedef std::vector<NodeId_t>::iterator NodeListIterator;
typedef std::vector<NodeId_t*> NodeListPtr;
typedef std::vector<NodeId_t*>::iterator NodeListPtrIterator;
typedef std::vector<ros::Publisher> PubList;
typedef ControlMessage ControlMessage_t;
}  // namespace task_net

ROS_STATIC_ASSERT(sizeof(task_net::State_t) == sizeof(task_net::State));
ROS_STATIC_ASSERT(sizeof(task_net::NodeBitmask) ==
  sizeof(task_net::NodeBitmask));

#define is_eq(a,b) (a.robot==b.robot&&a.type==b.type&&a.node==b.node)

namespace ros {
namespace message_traits {
// This type is fixed-size (24-bytes)
template<> struct IsFixedSize<task_net::NodeBitmask> : public TrueType {};
// This type is memcpyable
template<> struct IsSimple<task_net::NodeBitmask> : public TrueType {};

template<>
struct MD5Sum<task_net::NodeBitmask> {
  static const char* value() {
    return MD5Sum<robotics_task_tree_msgs::NodeBitmask>::value();
  }

  static const char* value(const task_net::NodeBitmask& m) {
    return MD5Sum<robotics_task_tree_msgs::NodeBitmask>::value();
  }
};

template<>
struct DataType<task_net::NodeBitmask> {
  static const char* value() {
    return DataType<robotics_task_tree_msgs::NodeBitmask>::value();
  }

  static const char* value(const task_net::NodeBitmask& m) {
    return DataType<robotics_task_tree_msgs::NodeBitmask>::value();
  }
};

template<>
struct Definition<task_net::NodeBitmask> {
  static const char* value() {
    return Definition<robotics_task_tree_msgs::NodeBitmask>::value();
  }

  static const char* value(const task_net::NodeBitmask& m) {
    return Definition<robotics_task_tree_msgs::NodeBitmask>::value();
  }
};

// This type is fixed-size (24-bytes)
template<> struct IsFixedSize<task_net::State_t> : public TrueType {};
// This type is memcpyable
template<> struct IsSimple<task_net::State_t> : public TrueType {};

template<>
struct MD5Sum<task_net::State_t> {
  static const char* value() {
    return MD5Sum<robotics_task_tree_msgs::State>::value();
  }

  static const char* value(const task_net::State_t& m) {
    return MD5Sum<robotics_task_tree_msgs::State>::value();
  }
};

template<>
struct DataType<task_net::State_t> {
  static const char* value() {
    return DataType<robotics_task_tree_msgs::State>::value();
  }

  static const char* value(const task_net::State_t& m) {
    return DataType<robotics_task_tree_msgs::State>::value();
  }
};

template<>
struct Definition<task_net::State_t> {
  static const char* value() {
    return Definition<robotics_task_tree_msgs::State>::value();
  }

  static const char* value(const task_net::State_t& m) {
    return Definition<robotics_task_tree_msgs::State>::value();
  }
};

////////////////////////////////////////////////////////////////////////////////
// Control Message Conversion for struct type agreement
////////////////////////////////////////////////////////////////////////////////
template<> struct IsFixedSize<task_net::ControlMessage_t> : public TrueType {};
// This type is memcpyable
template<> struct IsSimple<task_net::ControlMessage_t> : public TrueType {};

template<>
struct MD5Sum<task_net::ControlMessage_t> {
  static const char* value() {
    return MD5Sum<robotics_task_tree_msgs::ControlMessage>::value();
  }

  static const char* value(const task_net::ControlMessage_t& m) {
    return MD5Sum<robotics_task_tree_msgs::ControlMessage>::value();
  }
};

template<>
struct DataType<task_net::ControlMessage_t> {
  static const char* value() {
    return DataType<robotics_task_tree_msgs::ControlMessage>::value();
  }

  static const char* value(const task_net::ControlMessage_t& m) {
    return DataType<robotics_task_tree_msgs::ControlMessage>::value();
  }
};

template<>
struct Definition<task_net::ControlMessage_t> {
  static const char* value() {
    return Definition<robotics_task_tree_msgs::ControlMessage>::value();
  }

  static const char* value(const task_net::ControlMessage_t& m) {
    return Definition<robotics_task_tree_msgs::ControlMessage>::value();
  }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}  // namespace message_traits

/*TODO: KEEP UP TO DATE WITH STRUCT*/
namespace serialization {
template<>
struct Serializer<task_net::State_t> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.owner);
    stream.next(t.active);
    stream.next(t.done);
    stream.next(t.activation_level);
    stream.next(t.activation_potential);
    stream.next(t.peer_active);
    stream.next(t.peer_done);
    stream.next(t.parent_type);
    stream.next(t.suitability);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};

template<>
struct Serializer<task_net::NodeBitmask> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.type);
    stream.next(t.robot);
    stream.next(t.node);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};

////////////////////////////////////////////////////////////////////////////////
// Control Message Struct Serialization
////////////////////////////////////////////////////////////////////////////////
template<>
struct Serializer<task_net::ControlMessage_t> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.sender);
    stream.next(t.type);
    stream.next(t.activation_level);
    stream.next(t.activation_potential);
    stream.next(t.done);
    stream.next(t.active);
    stream.next(t.highest);
    stream.next(t.parent_type);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}  // namespace serialization
}  // namespace ros
#endif
