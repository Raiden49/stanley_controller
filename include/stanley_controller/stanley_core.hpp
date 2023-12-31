#ifndef STANLEY_CORE_HPP_
#define STANLEY_CORE_HPP_

#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "stanley_controller/stanley_tool.hpp"

namespace stanley_controller
{
class StanleyCore {
    public:
        double factor = 1;
        // double dt = 0.1;
        double speed = 0.5;

        /**
         * @brief Construction of Stanley core class
         * 
         * @param n node handle
        */
        StanleyCore(ros::NodeHandle& n) :n_(n) {};

        /**
         * @brief Callback function for subscribing odom pose
         * 
         * @param odom_msg odom point
        */
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

        /**
         * @brief Create a path for simulation
         * 
         * @param path path data
         * @return path vector
        */
        std::vector<path_type> CreatePath(visualization_msgs::Marker& path);

        /**
         * @brief Core function
        */
        void StanleyRun();
        ~StanleyCore() {};

    public:
        tf::Point odom_pos;
        double odom_yaw;
        double odom_linear_v, odom_angular_v;
        bool odom_callback_flag;    // avoid data error when using odom pose
        ros::Publisher cmd_vel_pub, marker_pub;
        ros::Subscriber odom_sub;

    private:
        ros::NodeHandle& n_;
        stanley_controller::StanleyTool math_tool;
};
}

#endif //STANLEY_CORE_HPP_