#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;

// Functions Get the matching points
void find_feature_matches(const Mat &img_1, const Mat &img_2,std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,std::vector<DMatch> &matches);

//Functions to Compute the E and H matrix
void get_pose_from_E(std::vector<KeyPoint> keypoint_01,
std::vector<KeyPoint>keypoint_02,
std::vector<DMatch> matches, Mat & R, Mat &t);
// Functions to Change the pixel points into camera normalization points
Point2d pixel2cam(const Point2d &p, const Mat &K);

// Functions traingulation Metering
void traingulationMetering( const vector <KeyPoint> & keypoint01,
const vector<KeyPoint> & keypoint02, const vector <DMatch> &matches,
const Mat& R, const Mat &t,vector<Point3d> &points);


int main(int argc, char*argv[]){
  
  // Step one : Find the matches point using    ORB-BRIEF  features
  if (argc != 3) {
    cout << "usage: pose_estimation_2d2d img1 img2" << endl;
    return 1;
  }
  //-- 读取图像
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  assert(img_1.data && img_2.data && "Can not load images!");

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << "Totally found" << matches.size() << " matches ! " << endl;

  // Step Two:   Compute the E and H matrix
  Mat R, t;
  get_pose_from_E(keypoints_1,keypoints_2,matches,R,t);

 // Step Three : validate the Epipolar constraint
     Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  //-- 验证E=t^R*scale
  Mat t_x =
    (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
      t.at<double>(2, 0), 0, -t.at<double>(0, 0),
      -t.at<double>(1, 0), t.at<double>(0, 0), 0);
  cout << "t^R=" << endl << t_x * R << endl;

    // Epipolar Constraint  : 
    //     tranpose(x2)* t^ * R *x1 =0 
     for (DMatch m: matches) {
    Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    Mat d = y2.t() * t_x * R * y1;
    // cout << "epipolar constraint (should be zero) =  " << d << endl;
     }
     //-- 三角化
  vector<Point3d> points3d;
  traingulationMetering(keypoints_1, keypoints_2, matches, R, t, points3d);
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


void get_pose_from_E(std::vector<KeyPoint> keypoint_01,
std::vector<KeyPoint>keypoint_02,
std::vector<DMatch> matches, Mat & R, Mat &t)
{
        // Camera 's inner parameter
         Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
        // get matches and points according to the matches
         vector<Point2f> points1;
         vector<Point2f> points2;
         for(int i =0;i<matches.size();i++){
                points1.push_back(keypoint_01[matches[i].queryIdx].pt);
                points2.push_back(keypoint_02[matches[i].queryIdx].pt);
         }
         
         // Using opencv to calculate the E matrix and F matrix

         Mat  f_matrix; Mat e_matrix;
         // using 8 points to calculate the F matrix
         f_matrix = findFundamentalMat(points1,points2,CV_FM_8POINT); 
         std::cout<< " Fundamental Matrix is:  "<<"\n"<<f_matrix<<std::endl;
        Point2d principal_point(325.1,249.7);   // TMU dataset calibrate
        const int focus_length = 521;
        e_matrix = findEssentialMat(points1,points2,focus_length,principal_point,RANSAC);
        std::cout<< " Essential Matrix is :"<<"\n" <<e_matrix<<std::endl;

        // Using opencv to calculate the H matrix
        Mat homography_matrix = findHomography(points1,points2,RANSAC,3,noArray(),200,0.99);
        std::cout<< " Homography Matrix is :"<<"\n" <<homography_matrix<<std::endl;


        // recover from E
        recoverPose(e_matrix,points1,points2,R,t,focus_length,principal_point);
        std::cout<<" R is :  "<<"\n"<<R<<std::endl;  // R is 3 by 3
        std::cout<<" t is :  "<<"\n"<<t<<std::endl;   //   t is 3  by 1
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}


void traingulationMetering( const vector <KeyPoint> & keypoint01,
const vector<KeyPoint> & keypoint02, const vector <DMatch> &matches,
const Mat& R, const Mat &t,vector<Point3d> &points){
 Mat T1 = (Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
  );

  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  vector<Point2f> pts_1, pts_2;
  for (DMatch m:matches) {
    // 将像素坐标转换至相机坐标
    pts_1.push_back(pixel2cam(keypoint01[m.queryIdx].pt, K));
    pts_2.push_back(pixel2cam(keypoint02[m.trainIdx].pt, K));  // keypoints 's training idx is corresponding to the training of cam 1
  }

  Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // 转换成非齐次坐标
  for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // 归一化
    Point3d p(
      x.at<float>(0, 0),
      x.at<float>(1, 0),
      x.at<float>(2, 0)
    );
    points.push_back(p);
  }

}