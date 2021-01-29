#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>

using namespace std;
using namespace cv;

// Functions Get the matching points
void find_feature_matches(const Mat &img_1, const Mat &img_2,std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,std::vector<DMatch> &matches);

// Functions to Change the pixel points into camera normalization points
Point2d pixel2cam(const Point2d &p, const Mat &K);

// using SVD for getting the ICP problem 
void solveIcp(const vector<Point3f> & pts1,const vector<Point3f> & pts2, Mat & R, Mat &t );


int main(int argc, char const *argv[])
{
 if (argc != 5) {
    cout << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << endl;
    return 1;
  }
 // ORB-BRIEF
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
// Keypoints and corresponding pairs
  std::vector<KeyPoint> keypoints_01;
  std::vector<KeyPoint> keypoints_02;
  std::vector<DMatch> matches;

  find_feature_matches(img_1,img_2, keypoints_01,keypoints_02,matches);
  cout<<"There is totally "<< matches.size()<<" matches found"<<endl;

    // 建立2D点对应的3D点
  Mat depth1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
  Mat depth2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); // Camera 内参
  vector<Point3f> pts1, pts2;

 for (DMatch m:matches) {
     //找到 Pairs对应的3D点坐标 depth
    ushort d1 = depth1.ptr<unsigned short>(int(keypoints_01[m.queryIdx].pt.y))[int(keypoints_01[m.queryIdx].pt.x)];   
    ushort d2 = depth2.ptr<unsigned short>(int(keypoints_02[m.trainIdx].pt.y))[int(keypoints_02[m.trainIdx].pt.x)];
    if (d1 == 0 || d2 == 0)   // bad depth
      continue;
    Point2d p1 = pixel2cam(keypoints_01[m.queryIdx].pt, K);   // Camera X'/Z', Y'/Z'
    Point2d p2 = pixel2cam(keypoints_02[m.trainIdx].pt, K);
    float dd1 = float(d1) / 5000.0;  // 定义一个单位长度 就是定义一个Z'
    float dd2 = float(d2) / 5000.0;
    pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));  // 定义一个单位长度 就是定义一个Z'
    pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
  }

  cout << "3d-3d pairs: " << pts1.size() << endl;
   // Solve ICP by SVD
   Mat R; Mat t;
   solveIcp(pts1,pts2,R,t);
   cout<< "R is "<<"\n"<<R<<endl;
   cout<<"t is "<<"\n"<<t<<endl;
 
// You can Also Using G2O for Bundle Adjustment

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


void solveIcp(const vector<Point3f> & pts1,const vector<Point3f> & pts2, Mat & R, Mat &t ){
 // 计算质心
 Point3f p1_mean, p2_mean;  // p1 and p2 are pairs
for(int i =0; i<pts1.size();i++){
    p1_mean+= pts1[i];
    p2_mean+= pts2[i];
    }
int nums_points=pts1.size();
p1_mean =  p1_mean / nums_points;
p2_mean = p2_mean / nums_points;
// 得到pts1 和 pts2 的去掉质心的坐标
vector <Point3f> pts1_after(nums_points); // 动态分配内存
vector <Point3f> pts2_after(nums_points); // 动态分配内存

for(int i =0;i<nums_points;i++){
pts1_after[i] = pts1[i] - p1_mean;
pts2_after[i] = pts2[i] - p2_mean;
    }
// Get W matrix 
Eigen::Matrix3d W = Eigen::Matrix3d::Zero(); // init with 0
for(int i =0;i<pts1.size();i++){
    W += Eigen::Vector3d(pts1_after[i].x, pts1_after[i].y,pts1_after[i].z) * Eigen::Vector3d(pts2_after[i].x, pts2_after[i].y, pts2_after[i].z).transpose();
 }
 cout<< " W matirx for SVD to Solving the R is "<<W<<endl;

 // 对 W 进行 SVD 分解
  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen:: Matrix3d matrix_R = U *(V.transpose());
  Eigen:: Vector3d t_ = Eigen::Vector3d(p1_mean.x,p1_mean.y,p1_mean.z) - matrix_R * Eigen::Vector3d(p2_mean.x,p2_mean.y,p2_mean.z) ;

  R = (Mat_<double>(3,3)<<matrix_R(0,0),matrix_R(0,1),matrix_R(0,2),
  matrix_R(1,0),matrix_R(1,1),matrix_R(1,2),
  matrix_R(2,0),matrix_R(2,1),matrix_R(2,2));
  t =(Mat_<double>(3,1)<<t_(0,0),t_(1,0),t_(2,0));

}