# Visual Odometry of Computer Vision 视觉里程计设计中的数学部分
Principile Algorithm of Visual Odometry of Computer Vision   ( Make sure You underStand LaTex )  
Chrome Latex Extention is required, If Not, Please via:  
https://blog.csdn.net/magicboom/article/details/113186548  

****Github Repo: https://github.com/Magicboomliu/VO****  
## Some basic mathematical concepts  一些基本的数学概念   
### **1.Camera Model 相机模型**  
假设P 的真实世界坐标为 $[X,Y,Z]{^T}$, 相机的成像坐标为 $[X',Y',Z']{^T}$, 焦距为 $f$ ,二者满足以下的关系:  
$$X' = \frac{X}{Z}$$  
$$Y' = \frac{Y}{Z}$$  
P像素坐标为 $[u,v]{^T}$ , $u$  、$v$和 $X'$、$Y'$之间又满足如下关系：  
$$\begin{cases} u =\alpha X' +c_x \\ u =\beta Y' +c_y \end{cases} $$     
令 $\alpha f = f_x, \beta f =f_y$,  
$$\begin{cases} u =f_x\frac{X}{Z} \\ v = f_y \frac{Y}{Z} \end{cases} $$     
以上的关系可以使用矩阵进行描述：  
$$
Z \begin{bmatrix}
   u\\
   v\\
   1 
  \end{bmatrix} =
  \begin{bmatrix}
      &f_x,&0,&c_x\\
     & 0,&f_y,&c_y\\
     & 0,&0,&1\\
  \end{bmatrix} 
  \begin{bmatrix}
      X\\
      Y\\
      Z
  \end{bmatrix} = KP
$$
K记录为相机内参矩阵，Camera Matrix.   
在现实世界中,P位置记录为$P_w$, $T$为变换矩阵：
$$
ZP_{uv} = Z \begin{bmatrix}
    u\\ v\\ 1
\end{bmatrix} = K(RP_w+t) = KTP_w
$$
其中 $TP_w$为相机坐标系下的做坐标，一般为归一化坐标$P_c$：  
$$
P_c '= TP_w, = \begin{bmatrix}
    X_c\\
    Y_c\\
    Z_c\\
\end{bmatrix}    
$$
$$
P_c = Normal(TP_w) = \begin{bmatrix}
    X_c/Z\\
    Y_c/Z\\
    Z_c/Z\\
\end{bmatrix}
$$

## 2.Epipolar Geometry 对极几何  
设第一个特征点为$p1$,第二个坐标点为$p2$, 初始状态下相机Pose 为 0，
则满足： 
$$\begin{cases} x_1p_1 = KP \\ x_2p_2 =K(RP+t) \end{cases} $$  
令：
$$\begin{cases}x_1 = K^{-1}p_1 \\ x_2 = K^{-1}p_2 \end{cases} $$  
则$x_1$，$x_2$ 满足对极约束：*（ram代表 Real antisymmetric matrix)*  
**$$x_2{^T}t_{ram} Rx_1=0$$**
如果用$p_1$，$p_2$表示为： $p_2^{T}K^{-T}t_{ram} RK^{-1}p_1=0$  
  
   **其中，记录 $E= t_{ram}R$为本质矩阵（Essential matrix）， $F= K^{-T}EK^{-1}$为基本矩阵。（Fundamental matrix）**  
   
  
补充： 若三维向量$a=\begin{bmatrix}
    a_1\\
    a_2\\
    a_3\\
\end{bmatrix}$ 那么a的反对称矩阵: 

$a_{ram}=\begin{bmatrix}
    0,&-a_3,&a_2\\
     a3,&0,&-a_1\\
     -a_2,&a1,&0\\
\end{bmatrix}$      

**VO的基本思路之一： 根据匹配特征点算E,F, 根据E,F用SVD或是Bundle Adjustment算R,t.**   

## 3 Lie algebra 李代数  
  李代数的存在是方便对旋转矩阵求微分， 从而达到优化的效果。因为旋转矩阵本身不满足“群”的性质： 比如 一个旋转矩阵$R_1$加上另外一个旋转矩阵$R_2$不能保证为旋转矩阵，因此很难进行微分运算， 因此我们把旋转矩阵映射到一个特殊的群上，使其能够满足进行微分运算的性质，这个群就是 “李群”。**李代数对应李群的正切空间，它描述了李群局部的导数。**   

  矩阵是可以微分的， 矩阵对时间$t$的微分为一个反对称(也叫斜对称)矩阵左乘它本身,**对于某个时刻的R(t)（李群空间），存在一个三维向量φ=（φ1，φ2，φ3）（李代数空间），用来描述R在t时刻的局部的导数**：  
  ![](https://img-blog.csdnimg.cn/img_convert/aa52c2778f21eacb12da2fc2fa9c42ec.png)    
  SO(3)上的原点 φ0 附近的正切空间上。这个φ正是李群大SO(3)对应的李代数小so(3),  即在原点附近，满足以下关系:   
  ![](https://img-blog.csdnimg.cn/img_convert/66e970649352a81f2bd9387f4918e590.png)  
  ### 李代数小so(3)是三维向量φ的集合，每个向量φi的反对称矩阵都可以表达李群(大SO(3))上旋转矩阵R的导数，而R和φ是一个指数映射关系。
  然后可以使使用Taylor公式对$R(t)$进行计算：  
  ![](https://img-blog.csdnimg.cn/img_convert/937b0fbc429238dd33d88e7ea07e3254.png)  
  ![](https://img-blog.csdnimg.cn/img_convert/16dd9896160a0abaed4d08259110251f.png)   
##  由此可以看出， so(3)本质上由旋转向量组成的的空间，这样我们可以说旋转矩阵的导数可以由其对应的旋转向量指定，指导如何在旋转矩阵中进行微积分运算。

## SO(3) SE(3)和 so(3) se(3)的转换关系： 
![](https://img-blog.csdnimg.cn/img_convert/6674b543e6c5708f52ee941fdbb3b4a0.png)  

## 矩阵优化的思路：  
1. 李代数表示Pose,使用李代数加法求导。（李代数求导模型）  
2.  使用李代数左乘，右乘微小扰动，对该扰动求导。（扰动模型）  

p为空间内部一点，R为Pose，我们对R求导。  

 （1）用李代数表示姿态，然后根据李代数加法来对李代数求导。即传统求导的思路，把增量直接定义在李代数上。  
![](https://img-blog.csdnimg.cn/20200720113407214.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQyNTE4OTU2,size_16,color_FFFFFF,t_70)   
(2) 对李群SO（3）左乘或者右乘微小扰动，然后对该扰动求导，称为左扰动和右扰动模型。即把增量扰动直接添加在李群上，然后使用李代数表示此扰动。  
![](https://img-blog.csdnimg.cn/20200720141933395.png)   

（3）SE(3)上面,含义是：考虑一个空间点P（需要是齐次坐标，否则维数不对），受到刚体变换T，得到TP：  
![](https://img-blog.csdnimg.cn/20200720143146117.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzQyNTE4OTU2,size_16,color_FFFFFF,t_70)   
上面最后一行矩阵除法，与矩阵乘法规则类似，只是乘号变成了除号。其使用一个4×1矩阵除以一个1×6矩阵得到一个4×6矩阵

### 引入李群李代数的意义：第一个是因为在欧式变换矩阵上不好定义导数，引入李群李代数使得导数定义变得自然合理；第二个是本来旋转矩阵与欧式变换矩阵具有本身的约束，使得将它们作为优化变量会引入额外约束，通过李群李代数可以使得问题变成一个无约束的优化问题（旋转变量有约束，因为旋转矩阵必须满足正交矩阵行列式为1，优化起来不方便）.  
![](https://img-blog.csdnimg.cn/img_convert/d5247cb5ae3d27d93a1d91739628f94d.png)    

## 4. 非线性优化问题  Nonlinear optimization problem  
 不能保证特征点绝对准确， 存在造成因素使得$R$,$
 t$求解出来不一定正确，因此需要Nonlinear optimization。
 * ### 常见的方法有高斯-牛顿（Gausss-Newton）G-N法 
 * ### 列文伯格-马夸尔特（Levenburg-Marquatt）LM法 

1. ## 高斯-牛顿（Gausss-Newton）G-N法   
假设优化函数为$f(x)$, 将该优化目标进行Taylor展开：
$$ 
f(x+\Delta x) = f(x)+ J(x)\Delta x
$$  
求解最小2乘问题： 
$$ 
\Delta x^* = argmin_{\Delta x^*} \frac{1}{2}\parallel {f(x)+J(x)\Delta x}\parallel ^2  
$$  
求解 $\Delta x$的导数使其为0.得到如下方程： 
$$
J(x)^{T}J(X)\Delta x = -J(x)f(x)
$$  
这个方程成为增量方程（Incremental equation), 也叫高斯牛顿方程（G-N equation,) 其中我们定义$J(x)^{T}J(X)$ 为 H(近似于2阶导数，Hessian Matrix),  $-J(x)f(x)$为 g（有些地方写成 b）, 则增量方程可以写成： 
$$
H\Delta x = g
$$  

现在给出G-N算法的流程：  
*  给定初始值 $x_0$  
*  计算此时的误差$f(x_0)$和对应的J(x).
*  计算H和g
*  计算$\Delta x$
*  Let $x' = x_0 + \Delta x$
*  重新计算误差$f(x_0)$和对应的J(x)，重复以上行为
*  当$\Delta x$足够小，停止。  
  
  ## 高斯-牛顿（Gausss-Newton）G-N法对应前段$Bundle Adjustment$  
  误差为重投影误差（Reprojection error）, 一般用于优化PnP问题或是ICP问题。  
  已知n个三维的空间点坐标P和投影p(像素坐标)，希望计算相机Pose 的 R 和 t, Pose 对应的se(3)为 $\xi$, 则根据相机模型，满足如下关系:  
  $$
s_iu_i = K exp(\xi_{ram})P_i
  $$
  重投影误差计算为：  
$$
\xi^*=argmin_{\xi}\frac{1}{2} \sum_{i=1}^{n} \parallel u_i -\frac{1}{s_i}K exp(\xi_{ram}) P_i \parallel ^2   
  $$  
  在优化前， 首先我们需要知道每个点的3维度相机坐标系下坐标：
  $$P' = exp(\xi_{ram}P)_{1:3} = [X' , Y', Z']$$ 
  这个可以根据相机模型去获得（根据相机坐标u ,v），因为： 
  $$\begin{cases}u = f_x\frac{X'}{Z'} \\ v = f_y\frac{Y'}{Z'} \end{cases} $$ 

  使用G-N方法，解的的$J(x)为一个 2 x 6 的雅克比矩阵：  
  $$ 
\frac{\partial e}{\partial \delta\xi} = - 
\begin{bmatrix}
    \frac{f_x}{Z'}, &0,&-\frac{f_xX'}{Z'^2},& -\frac{f_xX'Y'}{Z'^2}, &f_x +\frac{f_xX^2}{Z'^2}, &-\frac{f_xY'}{Z'} \\
    0, &\frac{f_y}{Z'}, &-\frac{f_yY'}{Z'^2},&-f_y -\frac{f_yY'^2}{Z'^2}, &-\frac{f_yX'Y'}{Z'^2}, &-\frac{f_yX'}{Z'} 
\end{bmatrix}
  $$ 
  根据这个J(x)可以计算H， 在根据H,f(x),J(x)计算$\Delta \xi$， 更新$\xi$，从而更新R和t。  
  BA算法出来优化Pose之外，还可以优化点P的坐标得到的J(x)是一个2 x3 的矩阵：  
   $$ 
\frac{\partial e}{\partial P} = - 
\begin{bmatrix}
 \frac{f_x}{Z'}, &0,&-\frac{f_xX'}{Z'^2} \\
 0, &\frac{f_y}{Z'},&-\frac{f_yY'}{Z'^2}
\end{bmatrix} R
  $$ 
然后计算$\Delta P$,更新P。  
下面展示 高翔博士 用G-N 做BA求解R和t 的 C++ Code  
$$ 
  \begin{bmatrix}
      &f_x,&0,&c_x\\
     & 0,&f_y,&c_y\\
     & 0,&0,&1\\
  \end{bmatrix} 
$$ 
为相机内参K， 第一步是// 将坐标转为相机坐标系下的坐标
````cpp
// 将坐标转为相机坐标系下的坐标
Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}
````
这里得到的Point2d其实是[$\frac{X'}{Z'}$,  $\frac{Y'}{Z'}$], 可以理解为用$Z'$归一化的相机坐标系下的坐标。  

接下来就是计算$\xi$
````cpp
void bundleAdjustmentGaussNewton(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose) {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  const int iterations = 10;
  double cost = 0, lastCost = 0;
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < iterations; iter++) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();  //这里之所以是 6 x 6 是因为J(x)TJ(X) = H, J(X)是2 x 6 
    Vector6d b = Vector6d::Zero(); // b = J(x)Tf(x), J(x)T是 6 x 2, f(x)是 x,y 2个维度的误差，所以是2 x 1.

    cost = 0;
    // compute cost
    for (int i = 0; i < points_3d.size(); i++) {
      Eigen::Vector3d pc = pose * points_3d[i];
      double inv_z = 1.0 / pc[2]; // 1/Z'
      double inv_z2 = inv_z * inv_z;  // 1/Z^2
      Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

      Eigen::Vector2d e = points_2d[i] - proj;  // 重投影误差

      cost += e.squaredNorm();
      Eigen::Matrix<double, 2, 6> J;  // 可以直接套结论的矩阵
      J << -fx * inv_z,
        0,
        fx * pc[0] * inv_z2,
        fx * pc[0] * pc[1] * inv_z2,
        -fx - fx * pc[0] * pc[0] * inv_z2,
        fx * pc[1] * inv_z,
        0,
        -fy * inv_z,
        fy * pc[1] * inv_z2,
        fy + fy * pc[1] * pc[1] * inv_z2,
        -fy * pc[0] * pc[1] * inv_z2,
        -fy * pc[0] * inv_z;

      H += J.transpose() * J;  // n点累加
      b += -J.transpose() * e;
    }

    Vector6d dx;
    dx = H.ldlt().solve(b); // 更新 se3

    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      // cost increase, update is not good
      cout << "cost: " << cost << ", last cost: " << lastCost << endl;
      break;
    }

    // update your estimation // 更新SE(3)
    pose = Sophus::SE3d::exp(dx) * pose; 
    lastCost = cost;

    cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
    if (dx.norm() < 1e-6) {  // 当需要更新的左乘se(3)足够小。
      // converge
      break;
    }
  }

  cout << "pose by g-n: \n" << pose.matrix() << endl;
}
````
在main （）函数中调用：  
````cpp
int main（int argc, char** argv）{
// ......前期求特征点或是三角测量 省略........
............
Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd)); // dd为深度， 第一个图知道3D
pts_2d.push_back(keypoints_2[m.trainIdx].pt);  // 第2个图知道2d， 这个是3d-2d，问题

// 使用 P-n-P求解
Mat r, t;
  solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
  Mat R;
  cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
  cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;
// 这是不用BA的解法
  cout << "R=" << endl << R << endl;
  cout << "t=" << endl << t << endl;

// 下面我们使用BA
 VecVector3d pts_3d_eigen;
  VecVector2d pts_2d_eigen;
  for (size_t i = 0; i < pts_3d.size(); ++i) {
    pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
    pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
  }

  cout << "Calling bundle adjustment by gauss newton" << endl;
  Sophus::SE3d pose_gn;
  bundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
cout<<"T="<<endl<<pose_gn.matirx()<<endl;
}
````
