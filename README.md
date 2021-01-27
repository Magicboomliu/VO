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
  ![](https://pic2.zhimg.com/v2-79c3071d633bfaddad1eba24195dce5d_r.jpg)    
  SO(3)上的原点 φ0 附近的正切空间上。这个φ正是李群大SO(3)对应的李代数小so(3),  即在原点附近，满足以下关系:   
  ![](https://pic3.zhimg.com/v2-2f7d78554ab00257cd58d58813ac96a2_r.jpg)  
  ### 李代数小so(3)是三维向量φ的集合，每个向量φi的反对称矩阵都可以表达李群(大SO(3))上旋转矩阵R的导数，而R和φ是一个指数映射关系。
  然后可以使使用Taylor公式对$R(t)$进行计算：  
  ![](https://pic1.zhimg.com/v2-8d19586b2e76bafed915af5c63c53c64_r.jpg)  
  ![](https://pic4.zhimg.com/v2-cc3027edb121b531208920c9bc4f29fb_r.jpg)   
##  由此可以看出， so(3)本质上由旋转向量组成的的空间，这样我们可以说旋转矩阵的导数可以由其对应的旋转向量指定，指导如何在旋转矩阵中进行微积分运算。

## SO(3) SE(3)和 so(3) se(3)的转换关系： 
![](https://pic1.zhimg.com/v2-e71634605ff59e642585155d7e8be4ac_r.jpg)  

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

### 引入李群李代数的意义：第一个是因为在欧式变换矩阵上不好定义导数，引入李群李代数使得导数定义变得自然合理；第二个是本来旋转矩阵与欧式变换矩阵具有本身的约束，使得将它们作为优化变量会引入额外约束，通过李群李代数可以使得问题变成一个无约束的优化问题.  
![](https://pic2.zhimg.com/v2-1097d5db89bedbd60b55ce5a23daef75_r.jpg)

