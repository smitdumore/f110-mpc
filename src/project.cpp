#include "f110-mpc/project.h"


project::~project()
{

}

project::project(ros::NodeHandle &nh) : occ_grid_(nh) , traj_plan_(nh) , traj_read_(nh)
{
    std::string pose_topic, scan_topic, drive_topic;

    nh_.getParam("/pose_topic", pose_topic);
    nh_.getParam("/scan_topic", scan_topic);
    nh_.getParam("/drive_topic", drive_topic);

    ros::NodeHandle nh_(nh);
    odom_sub_ = nh_.subscribe(pose_topic, 1, &project::OdomCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 1, &project::ScanCallback, this);
    current_pose_.position.x = 0;
    current_pose_.position.y = 0;
    current_pose_.position.z = 0;
    current_pose_.orientation.x = 0;
    current_pose_.orientation.y = 0;
    current_pose_.orientation.z = 0;
    current_pose_.orientation.w = 1;

    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);

    //traj_read_.ReadCSV("skirk");
    //global_path = traj_read_.waypoints_;

    traj_read_.ReadCSV("local_traj_50");
    traj_read_.Generate_Table();

    //traj_plan_.generate_traj_table();
}

void project::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // if (first_pose_estimate_)
    // {
    //     if (!first_scan_estimate_)
    //     {
             first_scan_estimate_ = true;
             //mpc_.UpdateScan(scan_msg);
    //     }
        //
        //State state(0.0,0.0,0.0);
        //sensor_msgs::LaserScan scan_msg_ = *scan_msg;

        //constraints_.FindHalfSpaces(state , scan_msg_);

        //occ_grid_.FillOccGrid(current_pose_, scan_msg);
        //occ_grid_.Visualize();
    //}
}


void project::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    
    traj_read_.Visualize();
    
}

Input project::GetNextInput()
{   
    //initially the size of current_inputs_ should be zero
    // inputs_idx_ intially has garbage value ??
    //debug both of these with ROS LOGS

    //if (inputs_idx_ >= current_inputs_.size())
    //{
        return Input(0.5,-0.05);           //v and steering
    //}

    // current_inputs_ is a vector of inputs
    //return current_inputs_[inputs_idx_];             //returns the input object at a certain index
}

