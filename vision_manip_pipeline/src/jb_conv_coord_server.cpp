#include "ros/ros.h"
#include "vision_manip_pipeline/Conv2DTo3D.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>

// global variable for point cloud
sensor_msgs::PointCloud2 pCloud;


bool checkConditions(float X, float Y, float Z) {
  if(  X < 10.0 && Y < 10.0 && Z < 10.0
    &&   X > -10.0 && Y > -10.0 && Z > -10.0
    && !std::isnan(X) && !std::isnan(Y) && !std::isnan(Z)  ){
    return true;
  }
  else{
    return false;
  }
}


void callback(const sensor_msgs::PointCloud2 pc){
  // set global point cloud to pc from kinect subscriber
  pCloud = pc;
}

bool convert2Dto3D(const sensor_msgs::PointCloud2 pCloud,
  vision_manip_pipeline::Conv2DTo3D::Request &req,
  vision_manip_pipeline::Conv2DTo3D::Response &res){

  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // initial x,y from request
  int x = req.x;
  int y = req.y;
  ROS_INFO("request: x=%ld, y=%ld", (long int)x, (long int)y);

  // initial X,Y,Z for response
  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  // spiral out from center pixel while value for x,y,z is out of bounds
  bool okay = false;
  int count = 0;
  int offset = 1;

  ROS_INFO("TEST");

  while( !okay ) {

    //-------------
    // get new Z from new x and y:
    //-------------
    ROS_INFO("\tTEST 1 ");

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = y*pCloud.row_step + x*pCloud.point_step;
    ROS_INFO("\tTEST 2 pos:%d row:%d point:%d", arrayPosition,pCloud.row_step,pCloud.point_step);

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
    ROS_INFO("\tTEST 3 ");

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));
    ROS_INFO("\tTEST 4 ");

    //-------------
    // test if new x,y,z in bounds:
    //-------------

    if ( checkConditions(X,Y,Z) ){
      okay = true;
    }
    else {
      std::cout << "count: " << count << " count mod 8: " << count%8 << std::endl;

      if( count % 8 == 0 ){
        x = req.x - offset;
        y = req.y - offset;
      }
      else if( count % 8 == 1 ){
        x = req.x;
        y = req.y - offset;
      }
      else if( count % 8 == 2 ){
        x = req.x + offset;
        y = req.y - offset;
      }
      else if( count % 8 == 3 ){
        x = req.x + offset;
        y = req.y;
      }
      else if( count % 8 == 4 ){
        x = req.x + offset;
        y = req.y + offset;
      }
      else if( count % 8 == 5 ){
        x = req.x;
        y = req.y + offset;
      }
      else if( count % 8 == 6 ){
        x = req.x - offset;
        y = req.y + offset;
      }
      else if( count % 8 == 7 ){
        x = req.x - offset;
        y = req.y;
        offset++;
      }
      ROS_INFO("\tnew x=%ld, new y=%ld", (long int)x, (long int)y );
      count++;
    }
  }

  // set final X,Y,Z
  res.newX = X;
  res.newY = Y;
  res.newZ = Z;

  ROS_INFO("solution: x=%f, y=%f, z=%f", (float)res.newX, (float)res.newY, (float)res.newZ);

  return true;
}


bool handle_conv_coord(vision_manip_pipeline::Conv2DTo3D::Request &req,
  vision_manip_pipeline::Conv2DTo3D::Response &res){

    convert2Dto3D(pCloud, req, res);
    return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "conv_coord");
  ros::NodeHandle n;

  // subscribe to kinect point cloud
  // TODO_PR2_TOPIC_CHANGE
  // ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, callback);
  // ros::Subscriber sub = n.subscribe("/kinect_head/depth_registered/points", 1000, callback);
  ros::Subscriber sub = n.subscribe("/kinect2/sd/points", 1000, callback);
  // ros::Subscriber sub = n.subscribe("/local/depth_registered/trans_points", 1000, callback);
  ros::ServiceServer service = n.advertiseService("conv_coord", handle_conv_coord);

  ROS_INFO("Ready to convert points.");
  ros::spin();

  return 0;
}
