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
    trajectories_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("valid_trajectories", 10);
    //best_traj_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("best_trajectory", 10);
    //valid_end_pub_ = nh_.advertise<visualization_msgs::Marker>("valid_end_point", 10);

    traj_read_.ReadCSV("skirk");
    global_path_ = traj_read_.waypoints_;

    dwa_traj_table_ = traj_plan_.generate_traj_table();
    //all dwa points are in base link 
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

        occ_grid_.FillOccGrid(current_pose_, scan_msg);
        occ_grid_.Visualize();
    //}
}


void project::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    //traj_plan_.visualize_dwa();       //visulaize all offline dwa trajs
    traj_read_.Visualize();             //visualize global path csv
    
    current_pose_ = odom_msg->pose.pose;
    geometry_msgs::Point p;
    
    for(int i=0; i < dwa_traj_table_.size(); i++)
    {
        int free_points_count = 0;
        for(int j=0; j < dwa_traj_table_.at(i).size(); j++)
        {
            
            geometry_msgs::Point base_link_point;
            base_link_point.x = dwa_traj_table_.at(i).at(j).x();
            base_link_point.y = dwa_traj_table_.at(i).at(j).y();

            std::pair<float, float> world_point = transforms_.CarPointToWorldPoint(base_link_point.x, base_link_point.y , current_pose_); 
            std::pair<int, int> occ_point = occ_grid_.WorldToOccupancy(world_point.first, world_point.second);

            if(occ_grid_.InGrid(occ_point.second, occ_point.first))
            {
                // SWAP
                if(occ_grid_.IsOccupied(occ_point.second, occ_point.first))
                {   
                    //this point is occupied
                }
                else
                {
                    free_points_count++;  
                }
            }
        }//one traj ends
        
        if(free_points_count == dwa_traj_table_.at(i).size())//num of discrete trajs
        {
            valid_traj_idx_.push_back(i);
            geometry_msgs::Point p;
            //converting all dwa base link END points to map frame
            std::pair<float, float> world_pair = transforms_.CarPointToWorldPoint(dwa_traj_table_.at(i).back().x(), dwa_traj_table_.at(i).back().y(), current_pose_);
            p.x = world_pair.first;
            p.y = world_pair.second;
            valid_end_points_.push_back(p);
        }   
    }

    if(valid_end_points_.size() == 0)
    {
        ROS_ERROR("NO VALID TRAJS");
        return;
    }
    
    // FOLLOW POINT //
    best_global_idx_ = traj_read_.get_best_global_idx(current_pose_);

    // DWA COST //  
    double min_dist =  numeric_limits<int>::max();

    for(int it=0 ; it < valid_end_points_.size(); it++)
    {
        double dist = pow (pow(valid_end_points_.at(it).x - global_path_.at(best_global_idx_).x() , 2) +
                          pow(valid_end_points_.at(it).y - global_path_.at(best_global_idx_).y() , 2), 0.5);

        if(dist < min_dist){
            min_dist = dist;
            best_traj_idx_ = it;     // in valid end point vector  
        }    
    }   
 
    valid_end_points_.clear();
    
    //IMP
    int best_trajectory_idx = valid_traj_idx_.at(best_traj_idx_);

    traj_plan_.Visualize_best_trajectory(best_trajectory_idx);

    valid_traj_idx_.clear();

    //follow this with mpc 
    // use a reached variable 
    // 
    
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

