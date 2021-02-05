#ifndef MAP_H
#define MAP_H
#include"simplevo/default_include.h"
#include"simplevo/frame.h"
#include"simplevo/mappoint.h"
namespace simplevo{
class Map{
public:
            typedef shared_ptr<Map> Ptr;
            unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks
            unordered_map<unsigned long, Frame::Ptr >     keyframes_;         // all key-frames
            Map() {}
            void insertKeyFrame( Frame::Ptr frame );  // add key-frames
            void insertMapPoint( MapPoint::Ptr map_point ); // add landmark of the frame
};
}
#endif