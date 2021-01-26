# Visual Odometry of Computer Vision 视觉里程计设计中的数学部分
Principile Algorithm of Visual Odometry of Computer Vision   ( Make sure You underStand LaTex )  
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