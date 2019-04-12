#include "ros/ros.h"
#include "vision_manip_pipeline/VisionManip.h"
#include <cstdlib>

int main(int argc, char **argv){

  ros::init(argc, argv, "vision_manip_cpp");
  if(argc != 2){
    ROS_INFO("usage: vision_manip 'object'");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<vision_manip_pipeline::VisionManip>("vision_manip");
  vision_manip_pipeline::VisionManip srv;
  srv.request.obj_name = argv[1];
  if(client.call(srv)){
    // ROS_INFO("NewX: %f NewY: %f NewZ: %f", (float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
    std::cout << "Approach Pose:      " << srv.response.approach_pose << '\n';
    std::cout << "Pick Pose:          " << srv.response.pick_pose << '\n';
    std::cout << "Score of Top Grasp: " << srv.response.score << '\n';
    std::cout << "Top Valid Grasp:    " << srv.response.grasp << '\n';
  }
  else{
    ROS_ERROR("Failed to call service vision_manip");
    return 1;
  }
  return 0;
}
