#include "stanley_controller/stanley_tool.hpp"

namespace stanley_controller 
{
int StanleyTool::GetMinDisIndex(path_type current_pose, 
        std::vector<path_type> path_vector) {

    double min_dis = INFINITY;
    int index = 0;
    for (int i = 0; i < path_vector.size(); i++) {
        double distance = GetDistance(path_vector[i], current_pose);
        if (distance < min_dis) {
            min_dis = distance;
            index = i;
        }
    }

    return index;
}

double StanleyTool::GetDistance(path_type pos1, path_type pos2) {

    double dis = sqrt(pow(pos1.first.first - pos2.first.first, 2) + 
            pow(pos1.first.second - pos2.first.second, 2));
    return dis;
}
}