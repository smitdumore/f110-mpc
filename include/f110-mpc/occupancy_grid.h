#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

//Uses a eigen boolean matrix to manage occupancy grid

class OccGrid
{   
    public:
        OccGrid();
        OccGrid(ros::NodeHandle &nh);

        virtual ~OccGrid();

        std::pair<int, int> WorldToOccupancy(std::pair<float, float> point);
        std::pair<int, int> WorldToOccupancy(float x, float y);
        std::pair<float,float> OccupancyToWorld(int row, int col);
        std::pair<float,float> OccupancyToWorld(std::pair<int,int> grid_point);
        std::pair<float, float> PolarToCartesian(float range, float angle);
        bool IsOccupied(float , float );

        void FillOccGrid(const geometry_msgs::Pose &pose_msg, const sensor_msgs::LaserScan::ConstPtr& scan_msg);

        bool InGrid(int col, int row);
        bool InGrid(std::pair<int, int> grid_point);
        bool CartesianInGrid(float x, float y);
        bool CartesianInGrid(std::pair<float, float> cart_point);

        void Visualize();
        int size();
    
    private:
    
        int size_;
        float discrete_;
        int grid_blocks_;
        float dilation_;
        std::pair<float, float> occ_offset_;
        Eigen::MatrixXf grid_;                   //underlying occupancy grid matrix
        ros::Publisher occ_pub_;
};

#endif  