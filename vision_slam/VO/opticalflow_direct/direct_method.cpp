#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};
// inline 修饰内联函数, 像素坐标到相机坐标
inline Eigen::Vector3d pixelToCam ( int u, int v, int d, float fx, float fy, float cx, float cy, float scale )
{
    float zz = float ( d ) /scale; // Give a measurment
    float x_cam = zz* ( u-cx ) /fx;
    float y_cam = zz* ( v-cy ) /fy;
    return Eigen::Vector3d ( x_cam, y_cam, zz );
}
// inline ， 相机坐标到像素坐标
inline Eigen::Vector2d camToPixel ( float x, float y, float z, float fx, float fy, float cx, float cy )
{
    float u = fx*x/z+cx;
    float v = fy*y/z+cy;
    return Eigen::Vector2d ( u,v );
}
// 直接法估计位姿
// 输入：测量值（空间点的灰度），新的灰度图，相机内参； 输出：相机位姿
// 返回：true为成功，false失败
bool poseEstimationDirect ( const vector<Measurement>& measurements, 
cv::Mat* gray, Eigen::Matrix3f& intrinsics, 
Eigen::Isometry3d& Tcw ); // Isometry3d --> 是 欧式 变换矩阵。 其实是一个 4 x 4 矩阵

int main(int argc, char const *argv[])
{


    /* code */
    return 0;
}
