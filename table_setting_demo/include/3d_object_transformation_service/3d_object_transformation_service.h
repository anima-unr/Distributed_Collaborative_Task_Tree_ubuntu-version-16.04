#ifndef OBJECT_TRANSFORMATION_SERVICE
#define OBJECT_TRANSFORMATION_SERVICE

#include "tabletop_segmenter/TabletopSegmentation.h"
#include "table_setting_demo/ObjectTransformation.h"
#include <boost/thread/thread.hpp>
#include "ros/ros.h"

namespace task_net {
typedef tabletop_segmenter::TabletopSegmentation::Response ObjectEnvironState_t;
class ObjectTransService {
 public:
  ObjectTransService(ros::NodeHandle *nh);
  virtual ~ObjectTransService();

  void GetState();

  bool GetObjectTransformation(
    table_setting_demo::ObjectTransformation::Request &req,
    table_setting_demo::ObjectTransformation::Response &res);
 private:
  ObjectEnvironState_t environment_state;
  boost::thread *state_thread;
  ros::ServiceServer service;

};
}  // namespace task_net
#endif  // OBJECT_TRANSFORMATION_SERVICE
