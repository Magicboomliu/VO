# ifndef   FRAME_H
#define  FRAME_H

#include"simplevo/default_include.h"
#include"simplevo/camera.h"

class Frame{

public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_ ; // id of this frame
    double time_stamp_; 
    Sophus::SE3d T_C_W ; // translation from world to frame
    Camera::Ptr camera_; // camera_model
    Mat color_; depth; // rgb image and rbgd image

}

#endif 