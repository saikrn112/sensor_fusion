#include <sensor_fusion/Fusion.hpp>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "fusion_node");
  ros::NodeHandle nh;
  Fusion::Ekf fuse(&nh);

}
