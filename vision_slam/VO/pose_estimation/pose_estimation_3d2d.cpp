#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>
using namespace std;
using namespace cv;

// Functions Get the matching points
void find_feature_matches(const Mat &img_1, const Mat &img_2,std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,std::vector<DMatch> &matches);

// Functions to Change the pixel points into camera normalization points
Point2d pixel2cam(const Point2d &p, const Mat &K);


int main(int argc, char const *argv[]){
    if(argc!=5){
        cout<<"Usage (bash): pose_estimation3d2d.cpp  <image01> <image02> <rgbd-image01-path>  <rgbd-image02-path> "<<endl;
       return 1;
    }
 //  Load Images
  Mat img_1 = imread(argv[1]);
  Mat img_2 = imread(argv[2]);
  assert(img_1.data && img_2.data && "Can not load images!");
 
 // Find keypoints matches using orb-brief
vector<KeyPoint> keypoints_1, keypoints_2;
vector<DMatch> matches;
find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
cout << "Totally found " << matches.size() << " matches" << endl;

//  Build 3d -2d  reflections

//  depth image is 16-bit unchar mono-channel picture
Mat rgbd_image01 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
Mat camera_K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);  
vector<Point3f> pts_3d;
vector<Point2f> pts_2d;
for(DMatch m :matches){
 // reference : Mat.ptr<unchar>(int a)[int b] : means the pixel in row a, col b.
 // get the keypoint's deep
ushort d = rgbd_image01.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];  
if(d==0) continue;   
float normalized_d = d /1000.0;
Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt,camera_K);
pts_3d.push_back(Point3f(p1.x*normalized_d,p1.y*normalized_d,normalized_d));  // keypint one's 3d 
pts_2d.push_back(keypoints_2[m.trainIdx].pt);      // keypoint2 :only 2d
}

// Solve PNP using OpenCV

Mat r, t;  // r is rotation vector , t is translation vector
solvePnP(pts_3d,pts_2d,camera_K,Mat(),r,t, false, cv::SOLVEPNP_EPNP);
Mat R;
cv::Rodrigues(r,R);

cout<<" Rotation Matrix R (3,3 ): "<<"\n"<<R<<endl;
cout<<"  translation vector  (3,1): "<<"\n"<<t<<endl;


return 0;
}


void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> match;
  //BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }
  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}
