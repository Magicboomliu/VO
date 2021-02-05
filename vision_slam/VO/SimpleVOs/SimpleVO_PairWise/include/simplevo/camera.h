# ifndef  CAMERA_H
#define CAMERA_H
#include"simplevo/default_include.h"
namespace simplevo
{
class Camera{
public:
    typedef std::shared_ptr<Camera> Ptr; // smart pointer
    float fx_, fy_,cx_,cy_,depth_scale_; // Camera Matrix intrinsics

    Camera();  // Construction Function

    Camera(float fx, float fy,float cx,float cy,float depth_scale =0) :
       fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {}

        // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const  Sophus::SE3d  & T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const Sophus::SE3d & T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    Vector3d pixel2world ( const Vector2d& p_p, const Sophus::SE3d & T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const Sophus::SE3d & T_c_w );
    };
} // namespace simplevo
#endif // Camera.h