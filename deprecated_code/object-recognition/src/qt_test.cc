#include "log.h"
#include "quad_tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>

int main(int argc, char *argv[]) {
  printf("Testing Quad Tree...\n");
  cv::Mat image(100,100, CV_8UC3, cv::Scalar(0,0,0));
  utils::AABB bbox(utils::Point(50.0, 50.0), 50.0);
  LOG_INFO("BBox: %f, %f, %f",
    bbox.center_.x_, bbox.center_.y_, bbox.half_dimension_);

  utils::QuadTree qt(bbox, 4);
  std::vector<utils::AABB> bounding_boxes;
  float x, y;
  for (int i = 0; i < 50; ++i) {
    x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 100.0;
    y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 100.0;
    qt.Insert(utils::Point(x, y));
  }
  qt.GetBounds(&bounding_boxes);
  printf("number of Bounding Boxes: %d\n", (int)bounding_boxes.size());

  cv::Point x1, x2;
  for (int i = 0; i < bounding_boxes.size(); ++i) {
    x1 = cv::Point(
      bounding_boxes[i].center_.x_ - bounding_boxes[i].half_dimension_,
      bounding_boxes[i].center_.y_ - bounding_boxes[i].half_dimension_);

    x2 = cv::Point(
      bounding_boxes[i].center_.x_ + bounding_boxes[i].half_dimension_,
      bounding_boxes[i].center_.y_ + bounding_boxes[i].half_dimension_);
    cv::rectangle(image, x1, x2, cv::Scalar(255,255,0), 2);
  }

  cv::imshow("QT_test", image);
  cv::waitKey(0);
  return 0;
}