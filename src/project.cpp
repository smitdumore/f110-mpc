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
    return;
    current_pose_ = odom_msg->pose.pose;

    std::vector<int> invalid_traj_idx;
    bool occ;

    visualization_msgs::Marker marker; 
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;
    //marker.id = k++;
    //marker.action = visualization_msgs::Marker::ADD;

    for(int i=0; i < dwa_traj_table_.size(); i++)
    {
        // occ = false;
        
        for(int j=0; j < dwa_traj_table_.at(i).size(); j++)
        {
            //ros::Duration(0.05).sleep();
            geometry_msgs::Point base_link_point;
            /*****************NO SWAP*****************/
            base_link_point.x = dwa_traj_table_.at(i).at(j).x();
            base_link_point.y = dwa_traj_table_.at(i).at(j).y();

            std::pair<float, float> world_point = transforms_.CarPointToWorldPoint(base_link_point.x, base_link_point.y , current_pose_);
            
            //publish this world point in map frame to check this transform
            std::pair<int, int> occ_point = occ_grid_.WorldToOccupancy(world_point.first, world_point.second);

            if(occ_grid_.InGrid(occ_point.first, occ_point.second))
            {
                if(occ_grid_.IsOccupied(occ_point.first, occ_point.second))
                {   
                    //occ = true;
                    //publish this point
                    
                    
                }
                else
                {
                    //point is free
                    //publish this point    
                    //convert occ to world
                    //std::pair<float, float> pair = occ_grid_.OccupancyToWorld(occ_point.first, occ_point.second);
                    marker.pose.position.x = world_point.first;
                    marker.pose.position.y = world_point.second;
                    marker.pose.position.z = 0.0;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.z = 0;
                    marker.pose.orientation.w = 0;

                    marker.scale.x = 0.3;
                    marker.scale.y = 0.3;
                    marker.scale.z = 0.3;
                    marker.color.a = 1.0; 
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    traj_read_.traj_pub_.publish(marker);
                    ros::Duration(0.05).sleep();

                }
            }
    
        }

        // if(occ)
        // {
        //     invalid_traj_idx.push_back(i);
        // }

        //traj_read_.traj_pub_.publish(marker);
        //marker.points.clear();
        
    }

    // for(auto it: invalid_traj_idx)
    // {
    //     std::cout << it << " ";      
    // }
    // std::cout << "\n";

    
        
        //marker.points.clear()
            

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

