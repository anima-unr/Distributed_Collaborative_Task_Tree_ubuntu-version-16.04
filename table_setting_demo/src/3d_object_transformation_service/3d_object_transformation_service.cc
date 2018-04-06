#include "3d_object_transformation_service/3d_object_transformation_service.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigenvalues>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

namespace task_net {
void StateUpdate(ObjectTransService *service) {
  while (true) {
    printf("Polling for table state\n");
    service->GetState();
    boost::this_thread::sleep(boost::posix_time::millisec(2000));
  }
}

ObjectTransService::ObjectTransService(ros::NodeHandle *nh) {
  // Start tabletop thread
  state_thread = new boost::thread(&StateUpdate, this);
}

ObjectTransService::~ObjectTransService() {}


void ObjectTransService::GetState() {
  tabletop_segmenter::TabletopSegmentation msg;
  if (ros::service::call("/tabletop_segmentation", msg)) {
    environment_state = msg.response;
  } else {
    ROS_ERROR("Tabletop Segmentation Service not responding");
  }
}

float PointDistance(cv::Point2f A, cv::Point2f B) {
  return std::sqrt(std::pow(A.x - B.x,2) + std::pow(A.y - B.y, 2));
}

// Function:    GetObjectPosition
// Description: Responsible for  taking the current state of the tabletop
//              and determine which object is represented by the image track
//              bounding box and return a tf transform from the space into world
bool ObjectTransService::GetObjectTransformation(
    table_setting_demo::ObjectTransformation::Request &req,
    table_setting_demo::ObjectTransformation::Response &res) {
  // Get point cloud data


  // for each cluster we need to compare it to the the objects
  cv::Mat mask;
  std::vector<cv::Point2f> object_centers(environment_state.masks.size());
  for (int i = 0; i < environment_state.masks.size(); ++i) {
    try {
      mask = cv_bridge::toCvCopy(environment_state.masks[i], "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("could not convert from '%s' to 'mono8'.", environment_state.masks[i].encoding.c_str());
    }

    // find the centroid of each image objects
    int pixel_count = 0;
    float x_sum = 0;
    float y_sum = 0;
    uint8_t *data = (uint8_t*)mask.data;
    for (int y = 0; y < mask.rows; ++y) {
      for (int x = 0; x < mask.cols; ++x) {
        if (data[y*mask.cols + x] > 0) {
          x_sum += x;
          y_sum += y;
          pixel_count++;
        }
      }
    }
    float xmean = x_sum / (float)pixel_count;
    float ymean = y_sum / (float)pixel_count;
    object_centers[i] = cv::Point2f(xmean, ymean);
  }

  // find the centroid of the bounding box messaged here
  float obj_meanx = req.x + req.w / 2.0;
  float obj_meany = req.y + req.h / 2.0;
  cv::Point2f object_centroid(obj_meanx, obj_meany);
  float minimum_dist = std::numeric_limits<float>::infinity();
  int index;
  // compare to the 
  std::vector<float> distances(object_centers.size());
  for (int i = 0; i < object_centers.size(); ++i) {
    distances[i] = PointDistance(object_centroid, object_centers[i]);
    if (distances[i] < minimum_dist) {
      index = i;
      minimum_dist = distances[i];
    }
  }

  // we know the group that it matches now as index

  // convert cluster from sensor_msgs::Pointcloud2 -> pcl::PointClout<PointT >
  pcl::PointCloud<pcl::PointXYZ> object_cloud;
  //Centroid
  Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;
  pcl::fromROSMsg(environment_state.clusters[index], object_cloud);

  // Transform point cloud into world coodinates
  tf::TransformListener tf_listen;
  tf::StampedTransform transform_cam_to_world;
  tf_listen.waitForTransform(
    environment_state.clusters[index].header.frame_id,
    "/base_link",
    ros::Time(0),
    ros::Duration(3.0));

  tf_listen.lookupTransform(
    environment_state.clusters[index].header.frame_id,
    "/base_link",
    ros::Time(0),
    transform_cam_to_world);

  pcl::PointCloud<pcl::PointXYZ> global_cloud;

  pcl_ros::transformPointCloud(
    object_cloud,
    global_cloud,
    transform_cam_to_world);


  // pcl::PointCloud<pcl::PointXYZ>::iterator it;
  // for (it = global_cloud.begin(); it != global_cloud.end(); ++it) {
  //   std::cout << "Point: " << *it << std::endl;
  // }
  // compute the orientation and position in 3D Space
  // compute first moment to get position
  pcl::compute3DCentroid(global_cloud, xyz_centroid);
  std::cout << xyz_centroid <<std::endl;

  // compute covariance matrix
  // pcl::computeCovarianceMatrix(global_cloud, xyz_centroid, covariance_matrix);


  // // obtain eigen vectors of covariance matrix
  // Eigen::EigenSolver<Eigen::Matrix3f> es;
  // es.compute(covariance_matrix);

  // // find the largest eigen value
  // Eigen::Vector3f eigenvalues = es.eigenvalues().real();
  // float max = -std::numeric_limits<float>::infinity();
  // int max_dex = -1;
  // float min = std::numeric_limits<float>::infinity();
  // int min_dex = -1;

  // for (int i = 0; i < 3; ++i) {
  //   if (max < eigenvalues(i)) {
  //     max = eigenvalues(i);
  //     max_dex = i;
  //   }
  //   if (min > eigenvalues(i)) {
  //     min = eigenvalues(i);
  //     min_dex = i;
  //   }
  // }
  // // sorted index
  // int sort[3];
  // sort[0] = max_dex;
  // sort[1] = 3 - (max_dex + min_dex);
  // sort[2] = min_dex;

  // Eigen::Vector3f x,y,z;
  // x = es.eigenvectors().col(sort[0]).real();
  // y = es.eigenvectors().col(sort[1]).real();
  // z = es.eigenvectors().col(sort[2]).real();
  // std::cout << "\nX: " << x << "\nY: " << y << "\nZ: " << z << std::endl;
  // Eigen::Vector3f wz;
  // wz << 0, 0, 1;
  // float d_wz[3];
  // d_wz[0] = wz.dot(x);
  // d_wz[1] = wz.dot(y);
  // d_wz[2] = wz.dot(z);
  // int max_dot_idx = 0;
  // float dot_max = d_wz[0];
  // for (int i = 1; i < 3; ++i) {
  //   if (dot_max < std::fabs(d_wz[i])) {
  //     dot_max = d_wz[i];
  //     max_dot_idx = i;
  //   }
  // }
  // if (dot_max < 0)

  // gminenerate rotation matrix from object space into world

  tf::Matrix3x3 rotation;
  rotation[0] = tf::Vector3(1,0,0);
  rotation[1] = tf::Vector3(0,1,0);
  rotation[2] = tf::Vector3(0,0,1);

  // tf::Vector3 position = tf::Vector3( xyz_centroid[0],
  //                                     xyz_centroid[1],
  //                                     xyz_centroid[2]);
  // printf("x: %f, y: %f, z: %f, w: %f\n",
  //   xyz_centroid(0),
  //   xyz_centroid(1),
  //   xyz_centroid(2),
  //   xyz_centroid(3));
  // position *= -1;
  tf::Quaternion orientation;
  rotation.getRotation(orientation);
  res.transform.header.frame_id = "base_link";
  res.transform.header.stamp = ros::Time::now();
  res.transform.transform.translation.x = xyz_centroid(0);
  res.transform.transform.translation.y = xyz_centroid(1);
  res.transform.transform.translation.z = 0;//xyz_centroid(2);
  res.transform.transform.rotation.x = orientation.x();
  res.transform.transform.rotation.y = orientation.y();
  res.transform.transform.rotation.z = orientation.z();
  res.transform.transform.rotation.w = orientation.w();

  // combine position and rotation matrix to generate transformation matrix
  printf("Object: %s\n", req.object.c_str());
  std::cout << res.transform.transform << std::endl;


  return true;
}
}