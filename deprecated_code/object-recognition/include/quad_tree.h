#ifndef QUAD_TREE_H_
#define QUAD_TREE_H_
#include <vector>
#include <stdint.h>
#include <opencv2/opencv.hpp>
namespace utils {
class Point {
 public:
  Point(float x = 0, float y = 0);
  Point(const Point *p);
  Point(cv::Point2f);
  Point& operator =(const Point &r);
  float x_, y_;
};
class AABB {
 public:
  AABB(Point center = Point(), float half_dimension = 0);
  uint32_t ContainsPoint(Point p);
  uint32_t IntersectsAABB(AABB bbox);

  Point center_;
  float half_dimension_;
};
class QuadTree {
 public:
  QuadTree(AABB bbox, uint32_t capacity);
  ~QuadTree();
  uint32_t Insert(Point point);
  std::vector<Point> QueryRange(AABB bbox);
  void GetBounds(std::vector<AABB> *bounds);
 private:
  uint32_t Subdivide();

  QuadTree *NW;
  QuadTree *NE;
  QuadTree *SW;
  QuadTree *SE;

  AABB bbox_;
  std::vector<Point> points_;
  uint32_t capacity_;
  bool divided_;
};
}
#endif  // QUAD_TREE_H_