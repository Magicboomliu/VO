#include <iostream>
#include<fstream>
#include<list>
#include<vector>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/video/tracking.hpp>
using namespace std;

// read dataset_contents from TUM
void read_dataset(string dataset_path){
   string associate_file_path = dataset_path+"/associate.txt";
   ifstream fin(associate_file_path);
   string rgb_file, depth_file,time_rgb,time_depth;
   list<cv::Point2f> keypoints; // list 方便delete
   cv::Mat color, depth,last_color;
   for(int index =0;index<100;index++){
       fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
       color = cv::imread(dataset_path+"/"+rgb_file);
       depth = cv::imread(dataset_path+"/"+depth_file,-1); // read 16-bit file
   // 对第一帧提取特征点
   if(index==0){
     vector<cv::KeyPoint> kps;
     cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
     detector->detect(color,kps);
     for(auto kp:kps){
         keypoints.push_back(kp.pt);
     }
     last_color = color;
     continue;  
   }
   // no data contiue. 有数据为止
   if ((color.data==nullptr) || (depth.data = nullptr)){
       continue;   }

// 对其他帧提取LK光流
vector<cv::Point2f> next_keypoints;
vector<cv::Point2f> prev_keypoints;
for(auto kp:keypoints){
    prev_keypoints.push_back(kp);
}
vector<unsigned char> status; // 记录追踪的状态
vector<float> error; 
cv::calcOpticalFlowPyrLK(last_color,color,prev_keypoints,next_keypoints,status,error);
// FixMe: This Part can do more things: to estimate the R and t.

// delete the points lost when tracking the optical flow
int i =0;
for(auto iter = keypoints.begin();iter!=keypoints.end();i++){
if(status[i]==0){
    iter = keypoints.erase(iter);
    continue;
}
*iter = next_keypoints[i];  // 更新当前的keypoints, 显示使用
iter++;
}
cout<<"tracking points found num: "<<keypoints.size()<<endl;
if(keypoints.size()==0){
    cout<<"lost all keypoints"<<endl;
    break;
}

// Draw Keypoints
cv::Mat img_show = color.clone();
for(auto kp:keypoints){
    cv::circle(img_show,kp,10,cv::Scalar(0,240,0),1);
}
cv::imshow("Corners",img_show);
cv::waitKey(0);
last_color = color;
   }
}



int main(int argc, char const *argv[])
{
  // Get  the dataset
  string dataset_path = argv[1];
 read_dataset(dataset_path);
    /* code */
    return 0;
}
