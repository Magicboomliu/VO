# VO
Principile Algorithm of Visual Odometry of Computer Vision   ( Make sure You underStand LaTex )    
If you cannot install chrome LaTex support, Please via:  
https://blog.csdn.net/magicboom/article/details/113186548
## Some basic mathematical concepts  一些基本的数学概念   
### **Camera Model 相机模型**  
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

