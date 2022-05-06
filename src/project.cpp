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
    trajectories_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("invalid_trajectories", 10);

    //traj_read_.ReadCSV("skirk");
    //global_path = traj_read_.waypoints_;

    // traj_read_.ReadCSV("local_traj_50");
    // traj_read_.Generate_Table();
    // dwa_traj_table_ = traj_read_.local_dwa_traj_table_; 

    dwa_traj_table_ = traj_plan_.generate_traj_table();
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
    traj_plan_.visualize_dwa();
    
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    //marker.id = k++;
    marker.action = visualization_msgs::Marker::ADD;


    current_pose_ = odom_msg->pose.pose;

    std::vector<int> valid_traj_idx;
    bool occ = false;

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
                /****************************************************
                 * ***********************SWAP ?? ********************
                *********row colums terminology*********************/
                if(occ_grid_.IsOccupied(occ_point.second, occ_point.first))
                {   
                    occ = true;
                }
                else
                {
                    geometry_msgs::Point p;
                    p.x = base_link_point.x;//world_point.first;
                    p.y = base_link_point.y;//world_point.second;
                    p.z = 0.0;
                    marker.points.push_back(p); 
                    free_points_count++;  
                }
            }
        }//one traj ends

        if(free_points_count == 20)
        {
            valid_traj_idx.push_back(i);
        }   
        occ = false; 
    }

    for(auto it: valid_traj_idx)
    {
        std::cout << it << " ";      
    }
    std::cout << "\n";

    marker.pose.position.x = 0.0;//current_pose_.position.x;
    marker.pose.position.y = 0.0;//current_pose_.position.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;//current_pose_.orientation.x;
    marker.pose.orientation.y = 0.0;//current_pose_.orientation.y;
    marker.pose.orientation.z = 0.0;//current_pose_.orientation.z;
    marker.pose.orientation.w = 0.0;//current_pose_.orientation.w;
    
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    traj_read_.traj_pub_.publish(marker);
    //return;
    /**********************************/
    visualization_msgs::MarkerArray traj_list;
    visualization_msgs::Marker traj;
    geometry_msgs::Point p;

    traj.header.frame_id = "base_link";
    traj.id = 0;                       ////////?
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.scale.x = traj.scale.y = 0.03;
    traj.scale.z = 0.03;
    traj.action = visualization_msgs::Marker::ADD;
    traj.pose.orientation.w = 1.0;
    traj.color.g = 1.0;
    traj.color.a = 1.0;



    for(int i=0; i < valid_traj_idx.size(); i++)
    {
        
        traj.id += 9;
        traj.points.clear();

        int curr_traj_idx = valid_traj_idx.at(i);
        std::vector<State> curr_traj = dwa_traj_table_.at(curr_traj_idx);

        for(int j=0; j <curr_traj.size() ; j++)
        {
            p.x = curr_traj.at(j).x();
            p.y = curr_traj.at(j).y();
            traj.points.push_back(p);
        }
        traj_list.markers.push_back(traj);
    }

    ROS_INFO("publishing traj viz");
    trajectories_viz_pub_.publish(traj_list);
    
    
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

