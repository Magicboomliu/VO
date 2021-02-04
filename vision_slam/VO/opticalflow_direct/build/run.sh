function KlOpticalFlow() {
 cmake ..
 make 
 ./optical_flow "TUM"
}
function sparse_direct_method(){
 cmake ..
 make 
 ./direct_method 
}

cmd=${1:-sparse_direct_method}
shift
$cmd "$@"