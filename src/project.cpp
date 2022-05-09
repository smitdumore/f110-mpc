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
    global_point_pub_ = nh_.advertise<visualization_msgs::Marker>("global_point", 10);
    best_traj_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("best_trajectory", 10);
    valid_end_pub_ = nh_.advertise<visualization_msgs::Marker>("valid_end_point", 10);

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
            //ROS_INFO("%d", i);
        }   
    }



    visualization_msgs::Marker end_marker;
    end_marker.header.frame_id = "map";
    end_marker.id = 1;
    end_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = 0.1;
    end_marker.color.b = 1.0;
    end_marker.color.a = 1.0;

    if(valid_end_points_.size() == 0)
    {
        ROS_ERROR("NO VALID TRAJS");
        return;
    }
    // all valid end points are in map frame
    for(int i=0 ; i < valid_end_points_.size(); i++)
    {
        p = valid_end_points_.at(i);
        end_marker.points.push_back(p);
    }

    valid_end_pub_.publish(end_marker);
    end_marker.points.clear();
    //ROS_WARN("valid end point size: %d", valid_end_points_.size()); 
    
    //all end points are now in map
    
    /* not used
    visualization_msgs::MarkerArray traj_list;
    visualization_msgs::Marker traj;
    geometry_msgs::Point p;

    traj.header.frame_id = "base_link";
    traj.id = 0;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.scale.x = traj.scale.y = 0.03;
    traj.scale.z = 0.03;
    traj.action = visualization_msgs::Marker::ADD;
    traj.pose.orientation.w = 1.0;
    traj.color.g = 1.0;
    traj.color.a = 1.0;

    for(int i=0; i < valid_traj_idx_.size(); i++)
    {
        
        traj.id += 9;
        traj.points.clear();

        int curr_traj_idx = valid_traj_idx_.at(i);
        std::vector<State> curr_traj = dwa_traj_table_.at(curr_traj_idx);

        for(int j=0; j <curr_traj.size() ; j++)
        {
            p.x = curr_traj.at(j).x();
            p.y = curr_traj.at(j).y();
            traj.points.push_back(p);
            if(j== curr_traj.size()-1)
            {
                //end points of each traj
                valid_end_points_.push_back(p);
            }
        }
        traj_list.markers.push_back(traj);
    }

    trajectories_viz_pub_.publish(traj_list);           //publishing all valid trajectories
    */
    // FOLLOW POINT //
    best_global_idx_ = traj_read_.get_best_global_idx(current_pose_);

    visualization_msgs::Marker global_marker;
    global_marker.id = 1;
    global_marker.header.frame_id = "map";
    global_marker.type = visualization_msgs::Marker::SPHERE;
    global_marker.scale.x = global_marker.scale.y = global_marker.scale.z = 0.2;
    global_marker.color.b = global_marker.color.r = 1.0;
    global_marker.color.g = 0.0;
    global_marker.color.a = 1.0;

    global_marker.pose.position.x = global_path_.at(best_global_idx_).x();
    global_marker.pose.position.y = global_path_.at(best_global_idx_).y();
    global_marker.pose.position.z = 0;

    global_point_pub_.publish(global_marker);       //publish best follow point

    // DWA COST //
    
    double min_dist =  numeric_limits<int>::max();

    for(int it=0 ; it < valid_end_points_.size(); it++)
    {
        //both are in map
        double dist = pow (pow(valid_end_points_.at(it).x - global_path_.at(best_global_idx_).x() , 2) +
                          pow(valid_end_points_.at(it).y - global_path_.at(best_global_idx_).y() , 2), 0.5);

        if(dist < min_dist){
            min_dist = dist;
            best_traj_idx_ = it;     // in valid end point vector  
        }    
    }   
 
    valid_end_points_.clear();
    
    //best_traj_idx_ is wrt valid_traj_idx_
    
    visualization_msgs::MarkerArray best_list;
    visualization_msgs::Marker best_traj;
    geometry_msgs::Point point;

    best_traj.header.frame_id = "base_link";
    best_traj.id = 1;
    best_traj.type = visualization_msgs::Marker::LINE_STRIP;
    best_traj.scale.x = best_traj.scale.y = 0.02;
    best_traj.action = visualization_msgs::Marker::ADD;
    best_traj.pose.orientation.w = 1.0;
    best_traj.color.r = 1.0;
    best_traj.color.a = 1.0;

    //IMP
    int best_trajectory_idx = valid_traj_idx_.at(best_traj_idx_);
    vector<State> best_trajectory = dwa_traj_table_.at(best_trajectory_idx);
    

    for(int i=0; i< best_trajectory.size(); i++)
    {
            point.x = best_trajectory.at(i).x();
            point.y = best_trajectory.at(i).y();
            best_traj.points.push_back(point);
    }
    best_list.markers.push_back(best_traj);

    best_traj_viz_pub_.publish(best_list);           //publihs best trajectory

    valid_traj_idx_.clear();
    
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

