#include "f110-mpc/project.h"


project::~project()
{

}

project::project(ros::NodeHandle &nh) : occ_grid_(nh) , constraints_(nh) , traj_(nh) , mpc_(nh)
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

    traj_.ReadCSV("fooxx_1point75");
    // bestMiniPath = traj_.waypoints_;
    

    ROS_INFO("Created project");
}

void project::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (first_pose_estimate_)
    {
        if (!first_scan_estimate_)
        {
           first_scan_estimate_ = true;
        }
        traj_.Visualize();

        float current_angle = Transforms::GetCarOrientation(current_pose_);
        State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);

        sensor_msgs::LaserScan scan_msg_ = *scan_msg;

        constraints_.FindHalfSpaces(current_state , scan_msg_);

        //occ_grid_.FillOccGrid(current_pose_, scan_msg);
        //occ_grid_.Visualize();
    }
}

void project::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    current_pose_ = odom_msg->pose.pose;
    if (!first_pose_estimate_)
    {
        first_pose_estimate_ = true;
    }
    float current_angle = Transforms::GetCarOrientation(current_pose_);
    State current_state(current_pose_.position.x, current_pose_.position.y, current_angle);
    
    if (first_scan_estimate_)
    {
        //Input input_to_pass = GetNextInput();
        //input_to_pass.set_v(4);

        std::vector<Input> ref_inputs;

        for(int i=0 ; i < mpc_.horizon(); i++)
        {
            State temp_state;
            temp_state.set_x( traj_.waypoints_.at(i).x() );
            temp_state.set_y( traj_.waypoints_.at(i).y() );
            temp_state.set_ori( 0.0 );

            Input temp_input;
            temp_input.set_v( 2.5 );
            temp_input.set_steer_ang( 0.0 );

            bestMiniPath.push_back(temp_state);
            ref_inputs.push_back(temp_input);
        }
        
        mpc_.initMPC(bestMiniPath,ref_inputs);

        ROS_WARN("SOLVED SIZE : %d" ,mpc_.solved_trajectory().size());
        //current_inputs_ = mpc_.solved_trajectory();
        //mpc_.Visualize();
        //inputs_idx_ = 0;
        
    }
}

Input project::GetNextInput()
{   
    
    if (inputs_idx_ >= current_inputs_.size())
    {
        ROS_ERROR("PUBLIHSING DEFAULT INPUTS");
        return Input(0.5,-0.05);           //v and steering
    }

    return current_inputs_[inputs_idx_];
}

/* TODO */ // add a drive loop