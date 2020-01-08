#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// global variable for point cloud
sensor_msgs::PointCloud2 pCloud;
sensor_msgs::PointCloud2 pCloud_out;
tf::TransformListener* listener;
ros::Publisher pub;

// https://answers.ros.org/question/205236/transforming-point-cloud/
void callback(const sensor_msgs::PointCloud2 pc){
  // set global point cloud to pc from kinect subscriber
  pCloud = pc;
  //ROS_WARN("AHHHHHx3!!!");
  // transform the point cloud
  tf::StampedTransform transform;
  try{
    // listener->lookupTransform( "/test", "/camera_depth_optical_frame",
    listener->lookupTransform( "/test", "/kinect2_rgb_optical_frame",
             ros::Time(0), transform);
    pcl_ros::transformPointCloud("/test", pCloud, pCloud_out, *listener);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.01).sleep();
  }
  //ROS_WARN("AHHHHH_4");
//ROS_WARN("AHHHHH 5?");
  // publish the point cloud
  pub.publish(pCloud_out);
  printf(".");
  fflush(stdout);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "orthoProjPointCloud");
  ros::NodeHandle n;
  ros::AsyncSpinner a(4);

  //ROS_WARN("AHHHHH");

  listener = new tf::TransformListener();
  pub = n.advertise<sensor_msgs::PointCloud2>("/local/depth_registered/trans_points", 1000);
  a.start();
  ros::Duration(1.0).sleep();
  //ROS_WARN("AHHHHH2!");
  // subscribe to point cloud
  //ros::Subscriber sub = n.subscribe("/local/depth_registered/points", 1000, callback);
  ros::Subscriber sub = n.subscribe("kinect2/qhd/points", 1000, callback);
  //ROS_WARN("AHHH 6 :D");
  
  ros::spin();


  return 0;
}
