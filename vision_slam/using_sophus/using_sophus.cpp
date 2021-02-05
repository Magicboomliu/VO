#include<iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<opencv2/core/eigen.hpp>
using namespace std;
using namespace Eigen;

int main(int argc, char const *argv[])
{
/////////////////////根据R或是q构建矩阵///////////////////////////
      // 沿Z轴转90度的旋转矩阵
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  // 或者四元数
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);              // Sophus::SO3d可以直接从旋转矩阵构造
  Sophus::SO3d SO3_q(q);              // 也可以通过四元数构造
  // 二者是等价的
  cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;   // 3 *3 Rotation Matrix
  cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl; //3 *3 Rotation Matrix
  cout << "they are equal" << endl;
  // cv::Mat r = (cv::Mat_<double>(3, 1) << 1, 2, 3);
  // cv::Mat R_;
  //   Vector3d t1(1, 0, 0);           // 沿X轴平移1
  // cv::Rodrigues(r,R_);
  // int row_mat = R_.rows;
  // int col_mat = R_.cols;
  // Eigen::MatrixXd R_eigen(row_mat,col_mat);
  // cv::cv2eigen(R_,R_eigen);
  // Sophus::SE3d se3d(R_eigen,t1 );
  // cout<<se3d.matrix()<<endl; // R is a mat, convert to Eigen for more operations


  ////////////////////////从李群获取李代数////////////////////////////////
   // 使用对数映射获得它的李代数
  Vector3d so3 = SO3_R.log();  // 3*1
  cout << "so3 = " << so3.transpose() << endl; // 1*3
  // hat 为向量到反对称矩阵
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl; // 表示R(t)导数
  // 相对的，vee为反对称到向量
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl; // 1*3
    /* code */

      // 增量扰动模型的更新
  Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多 , 3*1
  cout<<"update_so3 is :"<<"\n"<<update_so3<<endl;  
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;   // 左乘扰动模型
  cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;
  cout << "*******************************" << endl;



  // 对SE(3)操作大同小异
  Vector3d t(1, 0, 0);           // 沿X轴平移1
  Sophus::SE3d SE3_Rt(R, t);           // 从R,t构造SE(3)
  Sophus::SE3d SE3_qt(q, t);            // 从q,t构造SE(3)
  cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
  cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
  // 李代数se(3) 是一个六维向量，方便起见先typedef一下
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
  // 同样的，有hat和vee两个算符
  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

  // 最后，演示一下更新
  Vector6d update_se3; //更新量
  update_se3.setZero();
  update_se3(0, 0) = 1e-4; // 只更新第一个位置
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
  cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;
    return 0;
}
