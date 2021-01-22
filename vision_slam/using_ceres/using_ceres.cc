#include<iostream>
#include<opencv2/core/core.hpp>
#include<ceres/ceres.h>
#include<chrono>
using namespace std;

// define the cost function
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {} // struct 的构造函数
  // 残差的计算
  template<typename T>
  bool operator()(
    const T *const abc, // 模型参数，有3维, abc为输入维度， residual为输出维度
    T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }
  const double _x, _y;    // x,y数据
};

// Print the optimaization result
void printResult(double* para, size_t size){
 for (int i =0;i<size;i++){
     std::cout<<para[i]<<" ";
 }
 std::cout<<endl;
}

int main(int argc, char const *argv[])
{
     // 真实参数值 real, 估计参数值 estimate
  const double ar = 1.0, br = 2.0, cr = 1.0;
  double ae = 2.0, be = -1.0, ce = 5.0;      
  int dataAmount = 100;                                 // 数据点
  double w_sigma = 1.0;                        // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV随机数产生器
  
// 产生数据和噪声
 vector<double> x_data, y_data;  
  for (int i = 0; i < dataAmount; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }
  // 将要估计的参数放入一个vector中
  double abc[3] = {ae, be, ce};



// define a ceres problem
ceres::Problem problem;   
  for (int i = 0; i < dataAmount; i++) {
    problem.AddResidualBlock(     // 向问题中添加误差项
      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
      ),
      nullptr,            // 核函数，这里不使用，为空
      abc                 // 待估计参数
    );
  }
   // 配置求解器
  ceres::Solver::Options options;     // 这里有很多配置项可以填
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
  options.minimizer_progress_to_stdout = true;   // 输出到cout

  ceres::Solver::Summary summary;                // 优化信息
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();   // 记录当前的系统时间
  
  ceres::Solve(options, &problem, &summary);  // 开始优化
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();   // 记录优化完成的系统时间
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);  // 记录时间的耗费
  
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
  printResult(abc, 3);
    return 0;
}
