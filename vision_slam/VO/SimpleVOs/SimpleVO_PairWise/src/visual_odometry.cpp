#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>
#include <boost/timer.hpp>

#include"simplevo/config.h"
#include"simplevo/visual_odometry.h"

namespace simplevo{
VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}
VisualOdometry::~VisualOdometry()
{
}
// VO 对frame进行运动估计
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{
    switch ( state_ )
    {
    case INITIALIZING:  // 初始化状态
    {
        state_ = OK;
        curr_ = ref_ = frame;
        map_->insertKeyFrame ( frame ); 
        // extract features from first frame 
        extractKeyPoints();  
        computeDescriptors();
        // compute the 3d position of features in ref frame 
        setRef3DPoints();
        break;
    }
    case OK:
    {
        curr_ = frame; // 更新当前帧
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP(); // pnp : 3d-2d
        if ( checkEstimatedPose() == true ) // a good estimation
        {
            cout<<ref_->T_c_w_.matrix()<<endl;
            Eigen::MatrixXd culPoseMatrix(4,4);
            culPoseMatrix = T_c_r_estimated_.matrix() * ref_->T_c_w_.matrix();
            Eigen::MatrixXd rm(3,3);
            rm<<culPoseMatrix(0,0),culPoseMatrix(0,1),culPoseMatrix(0,2),
            culPoseMatrix(1,0),culPoseMatrix(1,1),culPoseMatrix(1,2),
            culPoseMatrix(2,0), culPoseMatrix(2,1), culPoseMatrix(2,2);
            Eigen::Vector3d tv(culPoseMatrix(0,3),culPoseMatrix(1,3),culPoseMatrix(2,3));
            curr_->T_c_w_ = Sophus::SE3d(rm,tv);
            cout<<"OKsss"<<endl;
            ref_ = curr_;
            setRef3DPoints();
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
        }
        else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"VO has lost."<<endl;
        break;
    }
    }
    return true;
}

void VisualOdometry::extractKeyPoints()
{
    orb_->detect ( curr_->color_, keypoints_curr_ ); // keypoints_curr: h
}
void VisualOdometry::computeDescriptors()
{
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
}
void VisualOdometry::featureMatching()
{
    // match desp_ref and desp_curr, use OpenCV's brute force match 
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher.match ( descriptors_ref_, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    feature_matches_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
}
void VisualOdometry::setRef3DPoints()
{
    // select the features with depth measurements 
    pts_3d_ref_.clear();
    descriptors_ref_ = Mat();
    for ( size_t i=0; i<keypoints_curr_.size(); i++ )
    {  
        double d = ref_->findDepth(keypoints_curr_[i]);     // FIXME
        if ( d > 0)
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

// Solve PNP problem
void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for ( cv::DMatch m:feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
    }
    // Camera Matrux
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );
    Mat rvec, tvec, inliers;
    cout<<"Pts3d Size is:  "<<pts3d.size()<<endl;
    cout<<"Pts2d Size is:  "<<pts2d.size()<<endl;
    cout<<pts3d[0].x<<"   "<<pts3d[0].y<<"    "<<pts3d[0].z<<endl;
    cout<<pts2d[0].x<<"  "<<pts2d[0].y<<"  "<<endl;
    bool result  = cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    
    // process the R and the T, rvec is the rotation vector, tvec is the translation vector
     Mat matrix_rotation;
     cv::Rodrigues(rvec,matrix_rotation);  // 3 x 3 rotation matrix
      int row_mat = matrix_rotation.rows;
      int col_mat = matrix_rotation.cols;
     Eigen::MatrixXd R_eigen(row_mat,col_mat);

      R_eigen<<matrix_rotation.at<double>(0,0),matrix_rotation.at<double>(0,1),matrix_rotation.at<double>(0,2),
     matrix_rotation.at<double>(1,0),matrix_rotation.at<double>(1,1),matrix_rotation.at<double>(1,2),
     matrix_rotation.at<double>(2,0),matrix_rotation.at<double>(2,1),matrix_rotation.at<double>(2,2);

    cout<<R_eigen.matrix()<<endl;
    T_c_r_estimated_ = Sophus::SE3d(
        R_eigen,
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    cout<<T_c_r_estimated_.matrix() <<endl;

}

bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}
bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();  
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}
// 匹配比较好的点作为KeyFrame
void VisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_ );
}
}