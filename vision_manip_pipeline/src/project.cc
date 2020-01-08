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

void transCloud();
void pubCloud();

void callback(const sensor_msgs::PointCloud2 pc){
  // set global point cloud to pc from kinect subscriber
  pCloud = pc;

  transCloud();

  // publish point cloud
  pubCloud();

}

// https://answers.ros.org/question/205236/transforming-point-cloud/
void transCloud() {


  tf::StampedTransform transform;
  try{
    listener->lookupTransform( "/test", "/camera_depth_optical_frame",  
             ros::Time(0), transform);
    }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  pcl_ros::transformPointCloud("/test", pCloud, pCloud_out, *listener);
  pub.publish(pCloud_out);

}


void pubCloud(){

  // generate new sensor message with pCloud_out as the daata

  // publish the point cloud to the new topic.....
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "orthoProjPointCloud");
  ros::NodeHandle n;
  listener = new tf::TransformListener();
  pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/trans_points", 1000);


  // subscribe to point cloud
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, callback);
  ros::spin();

  return 0;
}
