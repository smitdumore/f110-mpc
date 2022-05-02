#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H


#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "state.h"
#include "model.h"
//trajectory is a vector of states

class Traj_Plan
{
    public:
        Traj_Plan(ros::NodeHandle &nh_);
        ~Traj_Plan();

        void visualize_dwa();

    private:
        ros::Publisher trajectories_viz_pub_;
        std::vector<std::vector<std::vector<State>>> dwa_traj_table_;

        void generate_traj_table(); 
        
        double speed_max = 0.0;
        double steer_max = 0.0;
        int speed_discrete;
        int steer_discrete;
        int traj_discrete;
        double dt;
        Model model_;

};

#endif