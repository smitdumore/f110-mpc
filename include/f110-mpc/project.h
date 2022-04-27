#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <thread>
#include <chrono>
#include <mutex>

//custom files
#include "occupancy_grid.h"

class project
{
    public: 
        project(ros::NodeHandle &nh);
        virtual ~project();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher drive_pub_;

        geometry_msgs::Pose current_pose_;
        std::pair<float, float> occ_offset_;
        OccGrid occ_grid_;                      //occupancy grid object

        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};  

