#include "f110-mpc/occupancy_grid.h"

OccGrid::OccGrid(ros::NodeHandle &nh)
{

    nh.getParam("/occ_size", size_);
    nh.getParam("/occ_discrete", discrete_);
    nh.getParam("/occ_dilation", dilation_);           //obstacle inflation
    grid_blocks_ = size_/discrete_;
    grid_.resize(grid_blocks_, grid_blocks_);
    grid_ = Eigen::MatrixXf::Zero(grid_blocks_, grid_blocks_);
    occ_pub_ = nh.advertise<visualization_msgs::Marker>("occ_grid_marker", 100);

    ROS_INFO("occgrid created");
}

OccGrid::~OccGrid()
{
    ROS_INFO("Killing Occupancy grid");
}

std::pair<int, int> OccGrid::WorldToOccupancy(std::pair<float, float> point)
{
    return WorldToOccupancy(point.first, point.second);
}

std::pair<int, int> OccGrid::WorldToOccupancy(float x, float y)
{
    //x y coordinates to row major indices
    int occ_col = (x-occ_offset_.first) / discrete_ + grid_blocks_ / 2;
    int occ_row = (y-occ_offset_.second) / discrete_ + grid_blocks_ / 2;
    return std::pair<int, int>(occ_col,occ_row);
}

std::pair<float,float> OccGrid::OccupancyToWorld(int row, int col)
{
    float x = discrete_*(col-grid_blocks_/2)+occ_offset_.first;
    float y = discrete_*(row-grid_blocks_/2)+occ_offset_.second;
    return std::pair<float,float>(x,y);
}
std::pair<float,float> OccGrid::OccupancyToWorld(std::pair<int,int> grid_point)
{
    return OccupancyToWorld(grid_point.second, grid_point.first);
}


std::pair<float, float> OccGrid::PolarToCartesian(float range, float angle)
{
    std::pair<float, float> cartesian;
    cartesian.first = range * cos(angle);
    cartesian.second = range * sin(angle);
    return cartesian;
}

void OccGrid::FillOccGrid(const geometry_msgs::Pose &current_pose,const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    grid_ = Eigen::MatrixXf::Zero(grid_blocks_,grid_blocks_);

    //angle of the vehicle
    float current_angle = atan2(2 * current_pose.orientation.w * current_pose.orientation.z, 1 - 2 * current_pose.orientation.z * current_pose.orientation.z);

    //what is this  //origin of grid  ? 
    occ_offset_.first = current_pose.position.x + 0.275 * cos(current_angle);
    occ_offset_.second = current_pose.position.y + 0.275 * sin(current_angle);

    int num_scans = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment + 1;

    for (int ii = 0; ii < num_scans; ++ii)
    {
        //angle of the laser scan sweep
        float angle = scan_msg->angle_min + ii * scan_msg->angle_increment + current_angle;
        std::pair<float, float> cartesian = PolarToCartesian(scan_msg->ranges[ii], angle);
        cartesian.first += occ_offset_.first;
        cartesian.second += occ_offset_.second;
        
        for (float x_off = -dilation_; x_off <= dilation_; x_off += discrete_)
        {
            for (float y_off = -dilation_; y_off <= dilation_; y_off += discrete_)
            {
                std::pair<int, int> grid_point = WorldToOccupancy(cartesian.first + x_off, cartesian.second + y_off);
                if (InGrid(grid_point))
                {
                    grid_(grid_point.second, grid_point.first) = 1;
                }
            }
        }
    }
}

bool OccGrid::InGrid(int col, int row)
{
    if (col >= grid_blocks_ || col < 0 ||
        row >= grid_blocks_ || row < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
bool OccGrid::InGrid(std::pair<int, int> grid_point)
{
    return InGrid(grid_point.first, grid_point.second);
}
bool OccGrid::CartesianInGrid(float x, float y)
{
    std::pair<int, int> grid_point = WorldToOccupancy(x, y);
    return InGrid(grid_point);
}
bool OccGrid::CartesianInGrid(std::pair<float, float> grid_point)
{
    return CartesianInGrid(grid_point.first, grid_point.second);
}

void OccGrid::Visualize()
{

    std::vector<geometry_msgs::Point> occ_points;
    // occ_points.erase(occ_points.begin(),occ_points.end());
    for (int row = 0; row < grid_.rows(); ++row)
    {
        for (int col = 0; col < grid_.cols(); ++col)
        {
            if (grid_(row, col) == 1)
            {

                geometry_msgs::Point curr;
                std::pair<float, float> world_point = OccupancyToWorld(row, col);
                curr.x = world_point.first;
                curr.y = world_point.second;
                curr.z = 0.1;
                occ_points.push_back(curr);
            }
        }
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = occ_points;
    marker.scale.x = discrete_;
    marker.scale.y = discrete_;
    marker.scale.z = 0.1;
    marker.color.a = 0.6; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    occ_pub_.publish(marker);
}

int OccGrid::size()
{
    return size_;
}

bool OccGrid::IsOccupied(float x_ind, float y_ind)
{
    return grid_(x_ind, y_ind);
}