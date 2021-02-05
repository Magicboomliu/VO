#include "simplevo/frame.h"
namespace simplevo{
Frame::Frame(): id_(-1), time_stamp_(-1), camera_(nullptr){   
}
Frame::Frame ( long id, double time_stamp, Sophus::SE3d T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
{}
Frame::~Frame()
{}
// 添加一个Frame ， get a id ; extra is null ptr
Frame::Ptr Frame::createFrame(){
    static long factory_id = 0; 
    return Frame::Ptr( new Frame(factory_id++) );
}

// get keypoints's depth : Z ： 得到关键点的深度
double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];  // get depth
    if ( d!=0 )   // 如果深度不为 0 ，可以直接使用词典
    {
        return double(d)/camera_->depth_scale_;  // set  a scaloe
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};  
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];  // 以周围点的depth作为深度
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;   // -1.0 代表这一点没有深度
}

// Get Camera center
Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ ); // get camera pos
    if ( p_cam(2,0)<0 )  // under the camera
        return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ ); // get pixel
    return pixel(0,0)>0 && pixel(1,0)>0   
        && pixel(0,0)<color_.cols 
        && pixel(1,0)<color_.rows;
}


}