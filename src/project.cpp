#include "f110-mpc/project.h"


project::~project()
{

}

project::project(ros::NodeHandle &nh) : occ_grid_(nh) , constraints_(nh) , traj_read_(nh) , mpc_(nh)
{
    std::string pose_topic, scan_topic, drive_topic;

    nh_.getParam("/pose_topic", pose_topic);
    nh_.getParam("/scan_topic", scan_topic);
    nh_.getParam("/drive_topic", drive_topic);
    nh_.getParam("horizon", horizon_);

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

    traj_read_.ReadCSV("levine"); //skirk
    stateTrajectory_ = traj_read_.waypoints_;

    for(int i=0 ; i<horizon_; i++)
    {
        miniPath_.push_back(stateTrajectory_.at(i));
    }
    itr_ = horizon_;

    std::thread t(&project::DriveLoop, this);
    t.detach();


    ros::Duration(2.0).sleep();
}

void project::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (first_pose_estimate_)
    {
        if (!first_scan_estimate_)
        {
             first_scan_estimate_ = true;
             // updated scan message sent to mpc
             mpc_.UpdateScan(scan_msg);
        }
        
        float current_angle = Transforms::GetCarOrientation(current_pose_);
        State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);

        sensor_msgs::LaserScan scan_msg_ = *scan_msg;

        // scan message and current pose
        constraints_.FindHalfSpaces(current_state , scan_msg_);

        occ_grid_.FillOccGrid(current_pose_, scan_msg);
        occ_grid_.Visualize();
    }
}


void project::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    traj_read_.Visualize();

    current_pose_ = odom_msg->pose.pose;
    if (!first_pose_estimate_)
    {
        first_pose_estimate_ = true;
    }
    float current_angle = Transforms::GetCarOrientation(current_pose_);
    State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);
    ackermann_msgs::AckermannDriveStamped drive_msg;

    if (first_scan_estimate_)
    {
        
        Input input_to_pass = GetNextInput();
        input_to_pass.set_v(4.5);
    
        //end point of mini path 
        std::pair<float , float> end_point;
        end_point.first = miniPath_.back().x();
        end_point.second = miniPath_.back().y();
        
        std::pair<float , float> car_point;
        car_point.first = current_pose_.position.x;
        car_point.second = current_pose_.position.y;

        float dist = Transforms::CalcDist(car_point , end_point);

        if(dist < 0.4){                    //PROBLEM
            miniPath_.clear();
            for(int i=itr_ ; i < itr_ + horizon_ ; i++){
                if(i >= stateTrajectory_.size())
                {
                    ROS_ERROR("End");
                    break;
                }
                miniPath_.push_back( stateTrajectory_.at(i) );
            }
            itr_ += horizon_;
            //break;
        }

        // input_to_pass is the desired input ie (full throttle, zero steering)
        // current state
        // desired reference trajectory is miniPath_
        mpc_.Update(current_state ,input_to_pass, miniPath_); 
        
        current_inputs_ = mpc_.solved_trajectory();
        
        mpc_.Visualize();
        
        inputs_idx_ = 0;
        
    }
}

Input project::GetNextInput()
{   
    //initially the size of current_inputs_ should be zero
    // inputs_idx_ intially has garbage value ??
    //debug both of these with ROS LOGS

    if (inputs_idx_ >= current_inputs_.size())
    {
        return Input(0.5,-0.05);
    }

    return current_inputs_[inputs_idx_];
}


void project::DriveLoop()
{
    while (true)
    {
        if (first_pose_estimate_ && first_scan_estimate_)
        {
            ackermann_msgs::AckermannDriveStamped drive_msg;
            Input input = GetNextInput();
            
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.drive.speed = input.v() / 2.2;
            drive_msg.drive.steering_angle = input.steer_ang();
            drive_pub_.publish(drive_msg);
            int dt_ms = 2*mpc_.dt()*1000;
            inputs_idx_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        }
    }
}
