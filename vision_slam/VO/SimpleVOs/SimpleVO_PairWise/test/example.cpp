#include<iostream>
// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include"simplevo/visual_odometry.h"
#include "simplevo/config.h"



int main(int argc, char const *argv[]){
if ( argc != 2 ){
        cout<<"usage: example parameter_file"<<endl;
        return 1;
    }
    // get a config file 
    simplevo::Config::setParameterFile(argv[1]);
    simplevo::VisualOdometry::Ptr vo(new simplevo::VisualOdometry);  //declare a VO pointer
    string dataset_dir = simplevo::Config::get<string> ( "dataset_dir" );   // config dataset path
     ifstream fin ( dataset_dir+"/associate.txt" );
    int numsRbgFiles =0;
    
    if ( !fin )  // Make Sure Open the associate txt
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
 // Read the file until to the end
   while ( !fin.eof() ){
    string rgb_time, rgb_file, depth_time, depth_file;
    fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
    rgb_times.push_back ( atof ( rgb_time.c_str() ) );   // RBG images Timestamp
    depth_times.push_back ( atof ( depth_time.c_str() ) ); // Depth images Timestamps
    rgb_files.push_back ( dataset_dir+"/"+rgb_file );   // RBG FILE
    depth_files.push_back ( dataset_dir+"/"+depth_file ); // Depth FIle
    if ( fin.good() == false )
            break;
    }
    numsRbgFiles = rgb_files.size();
  // Build a Camera Model for Use and translation of coordinate 
  simplevo::Camera::Ptr camera_instance (new simplevo::Camera);
  for(int i =0;i<numsRbgFiles;i++){
      Mat color_image = cv::imread(rgb_files[i]);
      Mat depth_image = cv::imread(depth_files[i]);
      if((color_image.data==nullptr)||(depth_image.data==nullptr)){
          break;
      }
      // Build a Frame and Set Parameters
      simplevo::Frame::Ptr pFrame = simplevo::Frame::createFrame();
      pFrame->camera_ = camera_instance; 
      pFrame->color_ = color_image;
      pFrame->depth_ = depth_image;
      pFrame-> time_stamp_ = rgb_times[i];
     
    boost::timer timer;
    vo->addFrame(pFrame);
  }
    return 0;
}
