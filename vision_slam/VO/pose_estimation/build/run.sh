function pose_estimatation2d2d(){
cmake .. 
make 
./pose_estimation_2d2d 1.png 2.png
}

function pose_estimatation3d2d(){
cmake ..  
make 
./pose_estimation_3d2d 1.png 2.png 1_depth.png 2_depth.png
}

function pose_estimatation3d3d(){
 cmake ..
 make 
 ./pose_estimation_3d3d 1.png 2.png 1_depth.png 2_depth.png  

}

cmd=${1:-pose_estimatation3d3d}
shift
$cmd "$@"
