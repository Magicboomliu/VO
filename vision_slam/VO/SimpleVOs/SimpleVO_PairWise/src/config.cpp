#include "simplevo/config.h"

namespace simplevo
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ ); // read a yaml file
    if ( config_->file_.isOpened() == false )  // if no file
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()  // 析构函数
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}