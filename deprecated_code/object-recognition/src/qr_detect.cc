#include "qr_detect.h"
#include <stdint.h>
#include <map>
#include "quad_tree.h"
#include "log.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/flann/flann.hpp>
#include <iostream>
#include <cmath>
#include <queue>
#include <limits>
using namespace cv;
using namespace std;

namespace qr {
#define CV_QR_NORTH 0
#define CV_QR_EAST  1
#define CV_QR_SOUTH 2
#define CV_QR_WEST  3

struct ContourMoment {
  std::vector<cv::Point> contour;
  cv::Point2f moment;
};

struct PointCoord {
  PointCoord(int v_, cv::Point p_) {
    value = v_;
    p = p_;
  }
  int value;
  cv::Point p;
};

bool operator>(const PointCoord &a, const PointCoord &c) {
  return a.value > c.value;
}

bool operator>=(const PointCoord &a, const PointCoord &c) {
  return a.value >= c.value;
}

bool operator<(const PointCoord &a, const PointCoord &c) {
  return a.value < c.value;
}

bool operator<=(const PointCoord &a, const PointCoord &c) {
  return a.value <= c.value;
}

void QRGetContoursFromImage(cv::Mat image, Contour_t *contours,
    std::vector<cv::Vec4i> *hierarchy) {
  cv::Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));
  cv::Mat edge(image.size(), CV_MAKETYPE(image.depth(), 1));

  cv::cvtColor(image, gray, CV_RGB2GRAY);
  cv::Canny(gray, edge, 100, 200, 3);

  cv::findContours(edge, *contours, *hierarchy, cv::RETR_TREE,
    cv::CHAIN_APPROX_SIMPLE);
}

void QRGetContoursMassCenters(const Contour_t *contours,
    std::vector<cv::Point2f> *points) {
  std::vector<cv::Moments> mu(contours->size());
  for (int i = 0; i < contours->size(); ++i) {
    mu[i] = moments((*contours)[i], false);
    points->push_back(cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00));
  }
}

void QRCullContourCriteriaLengthRatio(const Contour_t *contours,
    const std::vector<cv::Vec4i> hierarchy,
    int32_t *mask) {
  float xmin, xmax, ymin, ymax;
  float e = 0.5;
  for (int i = 0; i < contours->size(); ++i) {
    if (mask[i]) {
      // Find Xmin, Xmax, Ymin, Ymax
      xmin = (*contours)[i][0].x;xmax = xmin;
      ymin = (*contours)[i][0].y;ymax = ymin;
      for (int j = 1; j < (*contours)[i].size(); ++j) {
        int x = (*contours)[i][j].x;
        int y = (*contours)[i][j].y;
        if (x > xmax)
          xmax = x;
        if (x < xmin)
          xmin = x;
        if (y > ymax)
          ymax = y;
        if (y < ymin)
          ymin = y;
      }
      // Determin Length ratio
      float r = (xmax - xmin)/(ymax - ymin);
      // printf("R: %f\n", r);
      if (!(1 - e < r && r < 1 + e))
        mask[i] = 0;
    }
  }
}

void QRCullContourCriteriaLength(const Contour_t *contours,
    const std::vector<cv::Vec4i> hierarchy,
    int32_t *mask) {
  int t = 25;
  for (int i = 0; i < contours->size(); ++i) {
    if ((*contours)[i].size() < t) {
      mask[i] = 0;
    }
  }
}

uint32_t CountContourChildren(const std::vector<cv::Vec4i> *hierarchy,
    int idx) {
  if ((*hierarchy)[idx][2] == -1) {
    return 0;
  } else {
    return CountContourChildren(hierarchy, (*hierarchy)[idx][2]) + 1;
  }
}

void QRCullContourCriteriaInterior(const Contour_t *contours,
    const std::vector<cv::Vec4i> hierarchy,
    int32_t *mask) {
  for (int i = 0; i < contours->size(); ++i) {
    if (mask[i] == 1) {
      if (CountContourChildren(&hierarchy, i) < 3) {
        mask[i] = 0;
      }
    }
  }
}

void QRCullContourCriteriaOverlap(const Contour_t *contours,
    const std::vector<cv::Vec4i> hierarchy,
    int32_t *mask) {
  float dist = 2;
  cv::Mat features;
  utils::QuadTree qt(
    utils::AABB(utils::Point(1280.0/2.0, 720.0/2), 1280.0/2.0), 4);
  std::vector<cv::Point2f> moments;
  std::vector<cv::Point2f> feature_points;
  // Compute moments
  QRGetContoursMassCenters(contours, &moments);

  for (int i = 0; i < moments.size(); ++i) {
    if (mask[i] == 1) {
      if (!(std::isnan(moments[i].x) || std::isnan(moments[i].y))){
        feature_points.push_back(moments[i]);
      } else {
        mask[i] = 0;
      }
    }
  }

  for (int i = 0; i < contours->size(); ++i) {
    if (mask[i] == 1) {
      qt.Insert(utils::Point(moments[i]));
    }
  }
  // Find Nearest Neghbors to discover identification markers
  std::vector<utils::Point> range;
  for (int i = 0; i < contours->size(); ++i) {
    if (mask[i] == 1) {
      range = qt.QueryRange(utils::AABB(utils::Point(moments[i]), dist));
      if (range.size() < 3) {
        mask[i] = 0;
      }
    }
  }
}

void QRApplyContourMask(Contour_t *contours, const int32_t *mask) {
  std::vector<std::vector<cv::Point> >::iterator it;
  Contour_t culled;
  int i = 0;
  for (it = contours->begin(); it != contours->end(); ++it) {
    if (mask[i]) {
      culled.push_back(*it);
    }
    i++;
  }
  (*contours) = culled;
}

void QRCullContours(Contour_t *contours, std::vector<cv::Vec4i> hierarchy) {
  int32_t *mask = new int32_t[contours->size()];
  for (int i = 0; i < contours->size(); ++i) {mask[i] = 1;}
  QRCullContourCriteriaLengthRatio(contours, hierarchy, mask);
  QRCullContourCriteriaLength(contours, hierarchy, mask);
  QRCullContourCriteriaOverlap(contours, hierarchy, mask);
  QRCullContourCriteriaInterior(contours, hierarchy, mask);
  QRApplyContourMask(contours, mask);
  delete [] mask;
}

cv::Mat QRGeneratePoints(Contour_t *contours) {
  std::vector<cv::Point2f> points;
  QRGetContoursMassCenters(contours, &points);
  utils::QuadTree qt(
    utils::AABB(utils::Point(640.0/2.0, 480/2.0), 640.0/2.0), 4);

  for (int i = 0; i < points.size(); ++i) {
    if (!std::isnan(points[i].x))
      qt.Insert(utils::Point(points[i]));
  }

  std::vector<utils::Point> range;
  std::vector<cv::Point2f> features;
  for (int i = 0; i < points.size(); ++i) {
    range = qt.QueryRange(utils::AABB(utils::Point(points[i]), 3.0));
    // find mean range
    cv::Point2f avg_point;
    avg_point.x = 0;
    avg_point.y = 0;
    for (int j = 0; j < range.size(); ++j) {
      avg_point.x += range[j].x_;
      avg_point.y += range[j].y_;
    }
    avg_point.x /= range.size();
    avg_point.y /= range.size();

    // Check if point is in features yet
    bool within = false;
    for (int j = 0; j < features.size(); ++j) {
      cv::Point2f diff = features[j] - avg_point;
      float dist = std::sqrt(std::pow(diff.x, 2) + std::pow(diff.y, 2));
      if (dist < 3)
        within = true;
    }
    if (!within) {
      features.push_back(avg_point);
    }
  }
  cv::Mat_<float> output(features.size(), 2);
  float *data = (float*)output.data;
  for (int i = 0; i < features.size(); ++i) {
    data[2*i]    = features[i].x;
    data[2*i +1] = features[i].y;
  }
  return output;
}

cv::Point ComputeIncenter(
    cv::Point2f A,
    cv::Point2f B,
    cv::Point2f C) {
  // float a,b,c;
  // cv::Point2f ab = A - B;
  // cv::Point2f bc = B - C;
  // cv::Point2f ca = C - A;
  // std::cout << "A:" << A << " - " << "B:" << B << " = " << ab << std::endl;
  // a = std::sqrt(pow(ab.x,2) + pow(ab.y,2));
  // b = std::sqrt(pow(bc.x,2) + pow(bc.y,2));
  // c = std::sqrt(pow(ca.x,2) + pow(ca.y,2));

  // cv::Point center;
  // center.x = (a * A.x + b * B.x + c * C.x) / (a + b + c);
  // center.y = (a * A.y + b * B.y + c * C.y) / (a + b + c);

  cv::Point center;
  center.x = (A.x + B.x + C.x) / 3.0;
  center.y = (A.y + B.y + C.y) / 3.0;
  return center;
}

std::vector<cv::Point> FindNMaxPoints(cv::Mat field, int num) {
  std::priority_queue<PointCoord, std::vector<PointCoord>, std::less<std::vector<PointCoord>::value_type> > queue;
  int *data = (int*)field.data;
  for (int i = 0; i < field.rows; ++i) {
    for (int j = 0; j < field.cols; ++j) {
      queue.push(PointCoord(data[field.cols * i + j], cv::Point(j, i)));
    }
  }
  std::vector<cv::Point> points;
  for (int i = 0; i < num; ++i) {
    points.push_back(queue.top().p);
    queue.pop();
  }
  return points;
}

// std::vector<std::vector<cv::Point2f> > QRDetectIdentifiers(cv::Mat image, Contour_t *ids) {
//   std::vector<cv::Vec4i> hierarchy;
//   std::vector<std::vector<cv::Point2f> > points;
//   QRGetContoursFromImage(image, ids, &hierarchy);
//   QRCullContours(ids, hierarchy);

//   // average contours into single mass centers
//   cv::Mat feature_points = QRGeneratePoints(ids);

//   if (feature_points.rows > 3) {
//     if (!feature_points.empty()) {
//       // Perform KNN to put points into groups of three
//       int knn = 3;
//       cv::Mat_<int> indices(feature_points.rows, knn);
//       cv::Mat_<float> dists(feature_points.rows, knn);

//       cv::flann::GenericIndex<cvflann::ChiSquareDistance<float> > index(feature_points,
//         cvflann::KDTreeIndexParams());

//       index.knnSearch(feature_points, indices, dists, knn, cvflann::SearchParams(feature_points.cols-1));

//       // group points into 3's
//       uint32_t num_groups = feature_points.rows / 3;
//       // Vote on Groups based on 3 point triangle incenter
//       std::vector<cv::Point> incenter_vector(feature_points.rows);
//       cv::Mat_<int> voting_field = cv::Mat::zeros(image.rows, image.cols, CV_32S);
//       for (int j = 0; j < feature_points.rows; ++j) {
//         // compute incenter
//         cv::Point2f A,B,C;
//         cv::Point incenter;
//         int a_idx, b_idx, c_idx;
//         a_idx = indices.at<int>(j, 0);
//         b_idx = indices.at<int>(j, 1);
//         c_idx = indices.at<int>(j, 2);

//         A.x = feature_points.at<float>(a_idx, 0);
//         A.y = feature_points.at<float>(a_idx, 1);

//         B.x = feature_points.at<float>(b_idx, 0);
//         B.y = feature_points.at<float>(b_idx, 1);

//         C.x = feature_points.at<float>(c_idx, 0);
//         C.y = feature_points.at<float>(c_idx, 1);

//         incenter = ComputeIncenter(A, B, C);
//         voting_field.at<uint32_t>(incenter.y, incenter.x) += 1;
//         incenter_vector.push_back(incenter);
//       }

//       std::vector<cv::Point> group_incenters = FindNMaxPoints(voting_field, num_groups);
//       points = std::vector<std::vector<cv::Point2f> >(group_incenters.size());
//       for (int i = 0; i < group_incenters.size(); ++i) {
//         for (int j = 0; j < feature_points.rows; ++j) {
//           if (incenter_vector[i] == group_incenters[j]) {
//             int a_idx = indices.at<int>(j,0);
//             int b_idx = indices.at<int>(j,1);
//             int c_idx = indices.at<int>(j,2);

//             points[i].push_back(
//                                 cv::Point2f(
//                                   feature_points.at<float>(a_idx, 0),
//                                   feature_points.at<float>(a_idx, 1)));
//             points[i].push_back(
//                                 cv::Point2f(
//                                   feature_points.at<float>(b_idx, 0),
//                                   feature_points.at<float>(b_idx, 1)));
//             points[i].push_back(
//                                 cv::Point2f(
//                                   feature_points.at<float>(c_idx, 0),
//                                   feature_points.at<float>(c_idx, 1)));
//             break;
//           }
//         }
//       }
//     }
//   } else if (feature_points.rows == 3) {
//     if (feature_points.rows > 0) {
//       float *data = (float*)feature_points.data;
//       std::vector<cv::Point2f> point;
//       for (int i = 0; i < feature_points.rows; ++i) {
//         point.push_back(cv::Point2f(data[2*i], data[2*i+1]));
//       }
//       points.push_back(point);
//     }
//   }
//   return points;
// }
float cv_distance(Point2f P, Point2f Q);          // Get Distance between two points
float cv_lineEquation(Point2f L, Point2f M, Point2f J);   // Perpendicular Distance of a Point J from line formed by Points L and M; Solution to equation of the line Val = ax+by+c 
float cv_lineSlope(Point2f L, Point2f M, int& alignement);  // Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
void cv_getVertices(vector<vector<Point> > contours, int c_id,float slope, vector<Point2f>& X);
void cv_updateCorner(Point2f P, Point2f ref ,float& baseline,  Point2f& corner);
void cv_updateCornerOr(int orientation, vector<Point2f> IN, vector<Point2f> &OUT);
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection);
float cross(Point2f v1,Point2f v2);

bool QRDetectIdentifiers(cv::Mat image, cv::Rect *roi, cv::Mat &qr_image) {
  // Creation of Intermediate 'Image' Objects required later
  Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));      // To hold Grayscale Image
  Mat edges(image.size(), CV_MAKETYPE(image.depth(), 1));     // To hold Grayscale Image
  Mat traces(image.size(), CV_8UC3);                // For Debug Visuals
  Mat qr,qr_raw,qr_gray,qr_thres;
      
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  int mark,A,B,C,top,right,bottom,median1,median2,outlier;
  float AB,BC,CA, dist,slope, areat,arear,areab, large, padding;
  
  int align,orientation;

  int DBG=1;            // Debug Flag

  int key = 0;
  traces = Scalar(0,0,0);
  qr_raw = Mat::zeros(100, 100, CV_8UC3 );
  qr = Mat::zeros(100, 100, CV_8UC3 );
  qr_gray = Mat::zeros(100, 100, CV_8UC1);
  qr_thres = Mat::zeros(100, 100, CV_8UC1);   

  cvtColor(image,gray,CV_RGB2GRAY);   // Convert Image captured from Image Input to GrayScale 
  Canny(gray, edges, 100 , 200, 3);   // Apply Canny edge detection on the gray image


  findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); // Find contours with hierarchy

  mark = 0;               // Reset all detected marker count for this frame

  // Get Moments for all Contours and the mass centers
  vector<Moments> mu(contours.size());
  vector<Point2f> mc(contours.size());

  for( int i = 0; i < contours.size(); i++ ) { 
    mu[i] = moments( contours[i], false ); 
    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
  }


  // Start processing the contour data

  // Find Three repeatedly enclosed contours A,B,C
  // NOTE: 1. Contour enclosing other contours is assumed to be the three Alignment markings of the QR code.
  // 2. Alternately, the Ratio of areas of the "concentric" squares can also be used for identifying base Alignment markers.
  // The below demonstrates the first method
  
  for( int i = 0; i < contours.size(); i++ ) {
    int k=i;
    int c=0;

    while (hierarchy[k][2] != -1) {
      k = hierarchy[k][2] ;
      c = c+1;
    }
    if (hierarchy[k][2] != -1)
      c = c+1;

    if (c >= 5) { 
      if (mark == 0)    A = i;
      else if  (mark == 1)  B = i;    // i.e., A is already found, assign current contour to B
      else if  (mark == 2)  C = i;    // i.e., A and B are already found, assign current contour to C
      mark = mark + 1;
    }
  }

  
  if (mark >= 3) {
    AB = cv_distance(mc[A],mc[B]);
    BC = cv_distance(mc[B],mc[C]);
    CA = cv_distance(mc[C],mc[A]);
    if (std::isnan(AB) || std::isnan(BC) || std::isnan(CA)) {
      return false;
    }
    if ( AB > BC && AB > CA ) {
      outlier = C; median1=A; median2=B;
    } else if ( CA > AB && CA > BC ) {
      outlier = B; median1=A; median2=C;
    } else if ( BC > AB && BC > CA ) {
      outlier = A;  median1=B; median2=C;
    }
          
    top = outlier;              // The obvious choice
  
    dist = cv_lineEquation(mc[median1], mc[median2], mc[outlier]);  // Get the Perpendicular distance of the outlier from the longest side      
    slope = cv_lineSlope(mc[median1], mc[median2],align);   // Also calculate the slope of the longest side
    
    // Now that we have the orientation of the line formed median1 & median2 and we also have the position of the outlier w.r.t. the line
    // Determine the 'right' and 'bottom' markers

    if (align == 0) {
      bottom = median1;
      right = median2;
    } else if (slope < 0 && dist < 0 ) {
      bottom = median1;
      right = median2;
      orientation = CV_QR_NORTH;
    } else if (slope > 0 && dist < 0 ) {
      right = median1;
      bottom = median2;
      orientation = CV_QR_EAST;
    } else if (slope < 0 && dist > 0 ) {
      right = median1;
      bottom = median2;
      orientation = CV_QR_SOUTH;
    } else if (slope > 0 && dist > 0 ) {
      bottom = median1;
      right = median2;
      orientation = CV_QR_WEST;
    }

    
    // To ensure any unintended values do not sneak up when QR code is not present
    float area_top,area_right, area_bottom;
    
    if (top < contours.size() && 
        right < contours.size() &&
        bottom < contours.size() &&
        contourArea(contours[top]) > 10 &&
        contourArea(contours[right]) > 10 &&
        contourArea(contours[bottom]) > 10 ) {

      vector<Point2f> L,M,O, tempL,tempM,tempO;
      Point2f N;  

      vector<Point2f> src,dst;    // src - Source Points basically the 4 end co-ordinates of the overlay image
                      // dst - Destination Points to transform overlay image  

      Mat warp_matrix;

      cv_getVertices(contours,top,slope,tempL);
      cv_getVertices(contours,right,slope,tempM);
      cv_getVertices(contours,bottom,slope,tempO);

      cv_updateCornerOr(orientation, tempL, L);       // Re-arrange marker corners w.r.t orientation of the QR code
      cv_updateCornerOr(orientation, tempM, M);       // Re-arrange marker corners w.r.t orientation of the QR code
      cv_updateCornerOr(orientation, tempO, O);       // Re-arrange marker corners w.r.t orientation of the QR code

      int iflag = getIntersectionPoint(M[1],M[2],O[3],O[2],N);

    
      src.push_back(L[0]);
      src.push_back(M[1]);
      src.push_back(N);
      src.push_back(O[3]);
      roi->x = L[0].x;
      roi->y = L[0].y;
      roi->width = N.x - roi->x;
      roi->width = N.y - roi->y;
      float minx = std::numeric_limits<float>::infinity();
      float miny = std::numeric_limits<float>::infinity();
      float maxx = -std::numeric_limits<float>::infinity();
      float maxy = -std::numeric_limits<float>::infinity();

      // find minumum x
      if (minx > L[0].x)
        minx = L[0].x;
      if (minx > M[1].x)
        minx = M[1].x;
      if (minx > N.x)
        minx = N.x;
      if (minx > O[3].x)
        minx = O[3].x;

      // find max x
      if (maxx < L[0].x)
        maxx = L[0].x;
      if (maxx < M[1].x)
        maxx = M[1].x;
      if (maxx < N.x)
        maxx = N.x;
      if (maxx < O[3].x)
        maxx = O[3].x;

      // find minumum x
      if (miny > L[0].y)
        miny = L[0].y;
      if (miny > M[1].y)
        miny = M[1].y;
      if (miny > N.y)
        miny = N.y;
      if (miny > O[3].y)
        miny = O[3].y;

      // find max x
      if (maxy < L[0].y)
        maxy = L[0].y;
      if (maxy < M[1].y)
        maxy = M[1].y;
      if (maxy < N.y)
        maxy = N.y;
      if (maxy < O[3].y)
        maxy = O[3].y;
      
      roi->x = minx;
      roi->y = miny;
      roi->width = maxx - minx;
      roi->height = maxy - miny;

      dst.push_back(Point2f(0,0));
      dst.push_back(Point2f(qr.cols,0));
      dst.push_back(Point2f(qr.cols, qr.rows));
      dst.push_back(Point2f(0, qr.rows));

      if (src.size() == 4 && dst.size() == 4 ) {
        warp_matrix = getPerspectiveTransform(src, dst);
        warpPerspective(image, qr_raw, warp_matrix, Size(qr.cols, qr.rows));
        copyMakeBorder( qr_raw, qr, 10, 10, 10, 10,BORDER_CONSTANT, Scalar(255,255,255) );
        
        cvtColor(qr,qr_gray,CV_RGB2GRAY);
        threshold(qr_gray, qr_thres, 127, 255, CV_THRESH_BINARY);
        qr_image = qr_thres;
        return true;
      }


      //Draw contours on the image
      drawContours( image, contours, top , Scalar(255,200,0), 2, 8, hierarchy, 0 );
      drawContours( image, contours, right , Scalar(0,0,255), 2, 8, hierarchy, 0 );
      drawContours( image, contours, bottom , Scalar(255,0,100), 2, 8, hierarchy, 0 );

      // Insert Debug instructions here
      if (DBG==1) {
        // Debug Prints
        // Visualizations for ease of understanding
        if (slope > 5){
          circle( traces, Point(10,20) , 5 ,  Scalar(0,0,255), -1, 8, 0 );
        } else if (slope < -5) {
          circle( traces, Point(10,20) , 5 ,  Scalar(255,255,255), -1, 8, 0 );
        }
          
        // Draw contours on Trace image for analysis  
        drawContours( traces, contours, top , Scalar(255,0,100), 1, 8, hierarchy, 0 );
        drawContours( traces, contours, right , Scalar(255,0,100), 1, 8, hierarchy, 0 );
        drawContours( traces, contours, bottom , Scalar(255,0,100), 1, 8, hierarchy, 0 );

        // Draw points (4 corners) on Trace image for each Identification marker  
        circle( traces, L[0], 2,  Scalar(255,255,0), -1, 8, 0 );
        circle( traces, L[1], 2,  Scalar(0,255,0), -1, 8, 0 );
        circle( traces, L[2], 2,  Scalar(0,0,255), -1, 8, 0 );
        circle( traces, L[3], 2,  Scalar(128,128,128), -1, 8, 0 );

        circle( traces, M[0], 2,  Scalar(255,255,0), -1, 8, 0 );
        circle( traces, M[1], 2,  Scalar(0,255,0), -1, 8, 0 );
        circle( traces, M[2], 2,  Scalar(0,0,255), -1, 8, 0 );
        circle( traces, M[3], 2,  Scalar(128,128,128), -1, 8, 0 );

        circle( traces, O[0], 2,  Scalar(255,255,0), -1, 8, 0 );
        circle( traces, O[1], 2,  Scalar(0,255,0), -1, 8, 0 );
        circle( traces, O[2], 2,  Scalar(0,0,255), -1, 8, 0 );
        circle( traces, O[3], 2,  Scalar(128,128,128), -1, 8, 0 );

        // Draw point of the estimated 4th Corner of (entire) QR Code
        circle( traces, N, 2,  Scalar(255,255,255), -1, 8, 0 );

        // Draw the lines used for estimating the 4th Corner of QR Code
        line(traces,M[1],N,Scalar(0,0,255),1,8,0);
        line(traces,O[3],N,Scalar(0,0,255),1,8,0);


        // Show the Orientation of the QR Code wrt to 2D Image Space
        int fontFace = FONT_HERSHEY_PLAIN;
         
        if (orientation == CV_QR_NORTH) {
          putText(traces, "NORTH", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
        } else if (orientation == CV_QR_EAST) {
          putText(traces, "EAST", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
        } else if (orientation == CV_QR_SOUTH) {
          putText(traces, "SOUTH", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
        } else if (orientation == CV_QR_WEST) {
          putText(traces, "WEST", Point(20,30), fontFace, 1, Scalar(0, 255, 0), 1, 8);
        }
      }
    }
  }

  return false;
}

// Function: Routine to get Distance between two points
// Description: Given 2 points, the function returns the distance

float cv_distance(Point2f P, Point2f Q)
{
  return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)) ; 
}


// Function: Perpendicular Distance of a Point J from line formed by Points L and M; Equation of the line ax+by+c=0
// Description: Given 3 points, the function derives the line quation of the first two points,
//    calculates and returns the perpendicular distance of the the 3rd point from this line.

float cv_lineEquation(Point2f L, Point2f M, Point2f J)
{
  float a,b,c,pdist;

  a = -((M.y - L.y) / (M.x - L.x));
  b = 1.0;
  c = (((M.y - L.y) /(M.x - L.x)) * L.x) - L.y;
  
  // Now that we have a, b, c from the equation ax + by + c, time to substitute (x,y) by values from the Point J

  pdist = (a * J.x + (b * J.y) + c) / sqrt((a * a) + (b * b));
  return pdist;
}

// Function: Slope of a line by two Points L and M on it; Slope of line, S = (x1 -x2) / (y1- y2)
// Description: Function returns the slope of the line formed by given 2 points, the alignement flag
//    indicates the line is vertical and the slope is infinity.

float cv_lineSlope(Point2f L, Point2f M, int& alignement)
{
  float dx,dy;
  dx = M.x - L.x;
  dy = M.y - L.y;
  
  if ( dy != 0)
  {  
    alignement = 1;
    return (dy / dx);
  }
  else        // Make sure we are not dividing by zero; so use 'alignement' flag
  {  
    alignement = 0;
    return 0.0;
  }
}



// Function: Routine to calculate 4 Corners of the Marker in Image Space using Region partitioning
// Theory: OpenCV Contours stores all points that describe it and these points lie the perimeter of the polygon.
//  The below function chooses the farthest points of the polygon since they form the vertices of that polygon,
//  exactly the points we are looking for. To choose the farthest point, the polygon is divided/partitioned into
//  4 regions equal regions using bounding box. Distance algorithm is applied between the centre of bounding box
//  every contour point in that region, the farthest point is deemed as the vertex of that region. Calculating
//  for all 4 regions we obtain the 4 corners of the polygon ( - quadrilateral).
void cv_getVertices(vector<vector<Point> > contours, int c_id, float slope, vector<Point2f>& quad)
{
  Rect box;
  box = boundingRect( contours[c_id]);
  
  Point2f M0,M1,M2,M3;
  Point2f A, B, C, D, W, X, Y, Z;

  A =  box.tl();
  B.x = box.br().x;
  B.y = box.tl().y;
  C = box.br();
  D.x = box.tl().x;
  D.y = box.br().y;


  W.x = (A.x + B.x) / 2;
  W.y = A.y;

  X.x = B.x;
  X.y = (B.y + C.y) / 2;

  Y.x = (C.x + D.x) / 2;
  Y.y = C.y;

  Z.x = D.x;
  Z.y = (D.y + A.y) / 2;

  float dmax[4];
  dmax[0]=0.0;
  dmax[1]=0.0;
  dmax[2]=0.0;
  dmax[3]=0.0;

  float pd1 = 0.0;
  float pd2 = 0.0;

  if (slope > 5 || slope < -5 )
  {

      for( int i = 0; i < contours[c_id].size(); i++ )
      {
    pd1 = cv_lineEquation(C,A,contours[c_id][i]); // Position of point w.r.t the diagonal AC 
    pd2 = cv_lineEquation(B,D,contours[c_id][i]); // Position of point w.r.t the diagonal BD

    if((pd1 >= 0.0) && (pd2 > 0.0))
    {
        cv_updateCorner(contours[c_id][i],W,dmax[1],M1);
    }
    else if((pd1 > 0.0) && (pd2 <= 0.0))
    {
        cv_updateCorner(contours[c_id][i],X,dmax[2],M2);
    }
    else if((pd1 <= 0.0) && (pd2 < 0.0))
    {
        cv_updateCorner(contours[c_id][i],Y,dmax[3],M3);
    }
    else if((pd1 < 0.0) && (pd2 >= 0.0))
    {
        cv_updateCorner(contours[c_id][i],Z,dmax[0],M0);
    }
    else
        continue;
             }
  }
  else
  {
    int halfx = (A.x + B.x) / 2;
    int halfy = (A.y + D.y) / 2;

    for( int i = 0; i < contours[c_id].size(); i++ )
    {
      if((contours[c_id][i].x < halfx) && (contours[c_id][i].y <= halfy))
      {
          cv_updateCorner(contours[c_id][i],C,dmax[2],M0);
      }
      else if((contours[c_id][i].x >= halfx) && (contours[c_id][i].y < halfy))
      {
          cv_updateCorner(contours[c_id][i],D,dmax[3],M1);
      }
      else if((contours[c_id][i].x > halfx) && (contours[c_id][i].y >= halfy))
      {
          cv_updateCorner(contours[c_id][i],A,dmax[0],M2);
      }
      else if((contours[c_id][i].x <= halfx) && (contours[c_id][i].y > halfy))
      {
          cv_updateCorner(contours[c_id][i],B,dmax[1],M3);
      }
        }
  }

  quad.push_back(M0);
  quad.push_back(M1);
  quad.push_back(M2);
  quad.push_back(M3);
  
}

// Function: Compare a point if it more far than previously recorded farthest distance
// Description: Farthest Point detection using reference point and baseline distance
void cv_updateCorner(Point2f P, Point2f ref , float& baseline,  Point2f& corner)
{
    float temp_dist;
    temp_dist = cv_distance(P,ref);

    if(temp_dist > baseline)
    {
        baseline = temp_dist;     // The farthest distance is the new baseline
        corner = P;           // P is now the farthest point
    }
  
}

// Function: Sequence the Corners wrt to the orientation of the QR Code
void cv_updateCornerOr(int orientation, vector<Point2f> IN,vector<Point2f> &OUT)
{
  Point2f M0,M1,M2,M3;
      if(orientation == CV_QR_NORTH)
  {
    M0 = IN[0];
    M1 = IN[1];
    M2 = IN[2];
    M3 = IN[3];
  }
  else if (orientation == CV_QR_EAST)
  {
    M0 = IN[1];
    M1 = IN[2];
    M2 = IN[3];
    M3 = IN[0];
  }
  else if (orientation == CV_QR_SOUTH)
  {
    M0 = IN[2];
    M1 = IN[3];
    M2 = IN[0];
    M3 = IN[1];
  }
  else if (orientation == CV_QR_WEST)
  {
    M0 = IN[3];
    M1 = IN[0];
    M2 = IN[1];
    M3 = IN[2];
  }

  OUT.push_back(M0);
  OUT.push_back(M1);
  OUT.push_back(M2);
  OUT.push_back(M3);
}

// Function: Get the Intersection Point of the lines formed by sets of two points
bool getIntersectionPoint(Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f& intersection)
{
    Point2f p = a1;
    Point2f q = b1;
    Point2f r(a2-a1);
    Point2f s(b2-b1);

    if(cross(r,s) == 0) {return false;}

    float t = cross(q-p,s)/cross(r,s);

    intersection = p + t*r;
    return true;
}

float cross(Point2f v1,Point2f v2)
{
    return v1.x*v2.y - v1.y*v2.x;
}
}