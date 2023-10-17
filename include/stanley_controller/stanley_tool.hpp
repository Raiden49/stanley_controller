#ifndef STANLEY_TOOL_HPP_
#define STANLEY_TOOL_HPP_

#include <cmath>
#include <iostream>
#include <vector>
#include <random>

using path_type = std::pair<std::pair<double, double>, double>;

namespace stanley_controller 
{
class StanleyTool {
    public:
        int GetMinDisIndex(path_type current_pose, 
                std::vector<path_type> path_vector);
        double GetDistance(path_type pos1, path_type pos2);
        StanleyTool() {};
        ~StanleyTool() {};

};
}

#endif // STANLEY_TOOL_HPP_