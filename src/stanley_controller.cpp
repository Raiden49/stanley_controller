#include "stanley_controller/stanley_core.hpp"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "stanley_controller");
    ros::NodeHandle n;
    auto node = new stanley_controller::StanleyCore(n);
    node->StanleyRun();

    return 0;
}