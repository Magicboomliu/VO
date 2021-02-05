# ifndef   FRAME_H
#define  FRAME_H
#include"simplevo/default_include.h"
#include"simplevo/camera.h"
namespace simplevo{
class Frame{
// data members
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                                       id_ ; // id of this frame
    double                                                      time_stamp_; 
    Sophus::SE3d                                        T_c_w_ ; // translation from world to frame
    Camera::Ptr                                            camera_; // camera_model
    Mat                                                             color_,depth_; // rgb image and rbgd image

public:
     Frame();
     Frame( long id, double time_stamp=0, Sophus::SE3d T_c_w=Sophus::SE3d(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
        // factory function
    static Frame::Ptr createFrame(); 
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    // Get Camera Center
    Vector3d getCamCenter() const;
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
};
}
#endif 