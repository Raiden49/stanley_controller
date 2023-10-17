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
        /**
         * @brief Get the index of min distance between robot pose and path
         * 
         * @param current_pose robot current pose
         * @param path_vector path point vector
         * @return index(int)
        */
        int GetMinDisIndex(path_type current_pose, 
                std::vector<path_type> path_vector);
                
        /**
         * @brief Get distance between pos1 and pos2
         * 
         * @param pos1 pos1 pose
         * @param pos2 pos2 pose
         * @return distance between pos1 and pos2
        */
        double GetDistance(path_type pos1, path_type pos2);

        /**
         * @brief Construction of Stanley math tool
        */
        StanleyTool() {};
        ~StanleyTool() {};

};
}

#endif // STANLEY_TOOL_HPP_