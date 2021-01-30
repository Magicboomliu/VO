function KlOpticalFlow() {
 cmake ..
 make 
 ./optical_flow "TUM"

}

cmd=${1:-KlOpticalFlow}
shift
$cmd "$@"