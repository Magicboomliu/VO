#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H
#include"simplevo/default_include.h"
#include<opencv2/xfeatures2d.hpp>
#include"simplevo/frame.h"
#include"simplevo/camera.h"
#include"simplevo/map.h"
#include"simplevo/mappoint.h"
namespace simplevo{
        class VisualOdometry{
            public :
                typedef shared_ptr<VisualOdometry> Ptr;
                // VO段匹配的状态
                enum VOState{
                    INITIALIZING =-1,
                    OK =0,
                    LOST
                };
                VOState        state_;  // current VO State
                Map::Ptr      map_;  // all frames and map points
                Frame::Ptr  ref_ ;// referenceing frame 
                Frame::Ptr  curr_; // currenet frame

               //*****  For features match ******//
                cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
                vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame 
                vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
                Mat                     descriptors_curr_;  // descriptor in current frame 
                Mat                     descriptors_ref_;   // descriptor in reference frame 
                vector<cv::DMatch>      feature_matches_;

                Sophus::SE3d T_c_r_estimated_;  // the estimated pose of current frame 
                int num_inliers_;        // number of inlier features in icp
                int num_lost_;           // number of lost times


                // parameters when matching 
                int num_of_features_;   // number of features
                double scale_factor_;   // scale in image pyramid
                int level_pyramid_;     // number of pyramid levels
                float match_ratio_;      // ratio for selecting  good matches
                int max_num_lost_;      // max number of continuous lost times, for change state of VO
                int min_inliers_;       // minimum inliers  , for check
    
                double key_frame_min_rot;   // minimal rotation of two key-frames, for check
                double key_frame_min_trans; // minimal translation of two key-frames, for check

        public: // functions 
                VisualOdometry();
                 ~VisualOdometry();
                bool addFrame( Frame::Ptr frame );      // add a new frame , 数据装入Frame类之后， 使用add frame（）估计pose, 内部调用inner functions
        
        protected:  
    // inner operation 
            void extractKeyPoints();
            void computeDescriptors(); 
            void featureMatching();
            void poseEstimationPnP(); 
            void setRef3DPoints();
    
            void addKeyFrame();
            bool checkEstimatedPose(); // 为了保证系统的鲁棒性， 检测 inlier的数目 和运动的大小， inlier数目不能太小， move不能太大
            bool checkKeyFrame();    

};
}
#endif
