#include "3d_object_transformation_service/3d_object_transformation_service.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "object_transformation_service");
  ros::NodeHandle nh;

  task_net::ObjectTransService service(&nh);

  ros::ServiceServer service_handle = nh.advertiseService(
    "object_transformation",
    &task_net::ObjectTransService::GetObjectTransformation,
    &service);

  ros::spin();
  return 0;
}