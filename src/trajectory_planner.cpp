#include "f110-mpc/trajectory_planner.h"

Traj_Plan::Traj_Plan(ros::NodeHandle &nh_)
{
    nh_.getParam("umax", speed_max);
    nh_.getParam("steer_max", steer_max);
    nh_.getParam("speed_discrete", speed_discrete);
    nh_.getParam("steer_discrete", steer_discrete);
    nh_.getParam("traj_discrete", traj_discrete);
    nh_.getParam("dt", dt);

    trajectories_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("dwa_trajectories", 10);
    ROS_WARN("Trajectory planner object created");
    ros::Duration(2.0).sleep();
 
}

Traj_Plan::~Traj_Plan()
{
    ROS_INFO("killing trajectory planner");
    dwa_traj_table_.clear();
}
/*
void Traj_Plan::generate_traj_table()
{
    dwa_traj_table_.clear();
    double dv = speed_max/speed_discrete;
    double ds = 2*+steer_max/steer_discrete; // 2 because minus and plus both
    State state;
    State new_state;
    Input input;

    std::vector<State> trajectory;
    std::vector<std::vector<State>> temp;

    for(int i=0; i < steer_discrete+1 ; i++)
    {
        temp.clear();
        for(int j=0; j < speed_discrete+1 ; j++)
        {
            trajectory.clear();

            double steer = -steer_max + i*ds;
            double speed = j*dv;
            state.set_x(0.0);state.set_y(0.0);state.set_ori(0.0);
            new_state.set_x(0.0);new_state.set_y(0.0);new_state.set_ori(0.0);
            
            input.set_v(speed);
            input.set_steer_ang(steer);
            trajectory.push_back(state);
            for(int k=0; k<traj_discrete; k++)
            {
                model_.simulate_dynamics(state, input, dt, new_state);
                trajectory.push_back(new_state);
                state = new_state;
            }
            temp.push_back(trajectory);
        }
        dwa_traj_table_.push_back(temp);
    }

    ROS_WARN("Generated Table");
    ros::Duration(2.0).sleep();

}//end of gen table

void Traj_Plan::visualize_dwa()
{
    int low = 0.3;
    int high = speed_max;

    double dv = speed_max/speed_discrete;
    visualization_msgs::MarkerArray traj_list;
    visualization_msgs::Marker traj;
    geometry_msgs::Point p;

    traj.header.frame_id = "base_link";
    traj.id = 0;                       ////////?
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.scale.x = traj.scale.y = 0.01;
    traj.scale.z = 0.02;
    traj.action = visualization_msgs::Marker::ADD;
    traj.pose.orientation.w = 1.0;
    traj.color.r = 1.0;
    traj.color.a = 1.0;

    for (int i=0; i<steer_discrete+1; i++) {
        for (int v_ind = low; v_ind <= high; v_ind++) {
            traj.points.clear();
            traj.id += 9;
            traj.color.b += 0.1;

            for (int j = 0; j < dwa_traj_table_[i][v_ind].size(); j++) {
                p.x = dwa_traj_table_[i][v_ind][j].x();
                p.y = dwa_traj_table_[i][v_ind][j].y();
                traj.points.push_back(p);
            }
            traj_list.markers.push_back(traj);
        }
    }
    ROS_INFO("publishing traj viz");
    trajectories_viz_pub_.publish(traj_list);
}
*/