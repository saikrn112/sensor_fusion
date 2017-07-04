#include <sensor_fusion/fusion.hpp>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "fusion_node");
  ros::NodeHandle nh;
  Fusion::RosIntegration::Ekf fuse(&nh);

}
