#include "quad_tree.h"
#include <cstddef>
#include <stdio.h>

namespace utils {
Point::Point(float x, float y) {
  x_ = x;
  y_ = y;
}

Point::Point(const Point *p) {
  x_ = p->x_;
  y_ = p->y_;
}

Point::Point(cv::Point2f p) {
  x_ = p.x;
  y_ = p.y;
}

Point& Point::operator=(const Point &r) {
  if (this == &r)
    return *this;
  this->x_ = r.x_;
  this->y_ = r.y_;
  return *this;
}

AABB::AABB(Point center, float half_dimension) {
  center_ = center;
  half_dimension_ = half_dimension;
}

uint32_t AABB::ContainsPoint(Point p) {
  if (p.x_ < (center_.x_ - half_dimension_) ||
      p.x_ >= (center_.x_ + half_dimension_))
    return -1;
  if (p.y_ < (center_.y_ - half_dimension_) ||
      p.y_ >= (center_.y_ + half_dimension_))
    return -1;
  return 1;
}

uint32_t AABB::IntersectsAABB(AABB bbox) {
  float ax_1,ay_1,ax_2,ay_2;
  float bx_1,by_1,bx_2,by_2;
  ax_1 = center_.x_ - half_dimension_; ay_1 = center_.y_ -
    half_dimension_;
  ax_2 = center_.x_ + half_dimension_; ay_2 = center_.y_ +
    half_dimension_;
  bx_1 = bbox.center_.x_ - bbox.half_dimension_;
  by_1 = bbox.center_.y_ - bbox.half_dimension_;
  bx_2 = bbox.center_.x_ + bbox.half_dimension_;
  by_2 = bbox.center_.y_ + bbox.half_dimension_;

  if (ax_1 < bx_2 && ax_2 > bx_1 && ay_1 < by_2 && ay_2 > by_1)
    return 1;
  return 0;
}

QuadTree::QuadTree(AABB bbox, uint32_t capacity) {
  bbox_ = bbox;
  capacity_  = capacity;
  NW = NULL;
  NE = NULL;
  SW = NULL;
  SE = NULL;
  divided_ = false;
}

QuadTree::~QuadTree() {
  delete NW;
  delete NE;
  delete SW;
  delete SE;
}

uint32_t QuadTree::Insert(Point point) {
  if (bbox_.ContainsPoint(point) == -1)
    return -1;
  if (points_.size() < capacity_ && divided_ == false) {
    points_.push_back(point);
    return 1;
  } else {
    if (NW == NULL)
      Subdivide();

    if (NW->Insert(point) == 1) return 1;
    if (NE->Insert(point) == 1) return 1;
    if (SW->Insert(point) == 1) return 1;
    if (SE->Insert(point) == 1) return 1;

    return -1;
  }
}

uint32_t QuadTree::Subdivide() {
  // Generate BBOZ for new qudrants
  AABB nw_bbox(
    Point(bbox_.center_.x_ - bbox_.half_dimension_/2.0,
      bbox_.center_.y_ + bbox_.half_dimension_/2.0),
    bbox_.half_dimension_/2.0);
  AABB ne_bbox(
    Point(bbox_.center_.x_ + bbox_.half_dimension_/2.0,
      bbox_.center_.y_ + bbox_.half_dimension_/2.0),
    bbox_.half_dimension_/2.0);
  AABB sw_bbox(
    Point(bbox_.center_.x_ - bbox_.half_dimension_/2.0,
      bbox_.center_.y_ - bbox_.half_dimension_/2.0),
    bbox_.half_dimension_/2.0);
  AABB se_bbox(
    Point(bbox_.center_.x_ + bbox_.half_dimension_/2.0,
      bbox_.center_.y_ - bbox_.half_dimension_/2.0),
    bbox_.half_dimension_/2.0);

  NW = new QuadTree(nw_bbox, capacity_);
  NE = new QuadTree(ne_bbox, capacity_);
  SW = new QuadTree(sw_bbox, capacity_);
  SE = new QuadTree(se_bbox, capacity_);

  for (int i = 0; i < points_.size(); ++i) {
    if        (NW->Insert(points_[i]) == 1) {
    } else if (NE->Insert(points_[i]) == 1) {
    } else if (SW->Insert(points_[i]) == 1) {
    } else if (SE->Insert(points_[i]) == 1) {
    }
  }
  divided_ = true;
  points_.clear();
  return 1;
}

std::vector<Point> QuadTree::QueryRange(AABB bbox) {
  std::vector<Point> range_points;
  if (!bbox_.IntersectsAABB(bbox))
    return range_points;
  for (int i = 0; i < points_.size(); ++i) {
    if (bbox.ContainsPoint(points_[i]))
      range_points.push_back(points_[i]);
  }
  if (NW == NULL)
    return range_points;
  std::vector<Point> temp_range;
  temp_range = NW->QueryRange(bbox);
  range_points.insert(range_points.end(), temp_range.begin(), temp_range.end());
  temp_range.clear();

  temp_range = NE->QueryRange(bbox);
  range_points.insert(range_points.end(), temp_range.begin(), temp_range.end());
  temp_range.clear();

  temp_range = SW->QueryRange(bbox);
  range_points.insert(range_points.end(), temp_range.begin(), temp_range.end());
  temp_range.clear();

  temp_range = SE->QueryRange(bbox);
  range_points.insert(range_points.end(), temp_range.begin(), temp_range.end());
  temp_range.clear();

  return range_points;
}

void QuadTree::GetBounds(std::vector<AABB> *bounds) {
  bounds->push_back(bbox_);

  if (NW != NULL) {
    NW->GetBounds(bounds);
    NE->GetBounds(bounds);
    SW->GetBounds(bounds);
    SE->GetBounds(bounds);
  }
}
}  // namespace utils