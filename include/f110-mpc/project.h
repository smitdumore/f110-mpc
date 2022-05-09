#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "OsqpEigen/OsqpEigen.h"
#include <thread>
#include <chrono>
#include <mutex>

//custom files
#include "occupancy_grid.h"
#include "input.h"
#include "state.h"
#include "transforms.h"
#include "trajectory.h"
#include "constraints.h"
#include "mpc.h"
#include "trajectory_planner.h"

class project
{
    public: 
        project(ros::NodeHandle &nh);
        virtual ~project();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Subscriber odom_sub_;
        ros::Publisher drive_pub_;
        ros::Publisher trajectories_viz_pub_;
        ros::Publisher valid_end_pub_;

        bool first_pose_estimate_ = false;
        bool first_scan_estimate_ = false;

        geometry_msgs::Pose current_pose_;
        std::pair<float, float> occ_offset_;

        //all objects
        OccGrid occ_grid_;                      
        Constraints constraints_;               
        Trajectory traj_read_;                
        MPC mpc_;
        Traj_Plan traj_plan_;
        Transforms transforms_;

        //mpc specific
        std::vector<Input> current_inputs_;
    
        //dwa specific
        std::vector<State> global_path_;
        std::vector<std::vector<State>> dwa_traj_table_;
        std::vector<int> valid_traj_idx_;
        std::vector<geometry_msgs::Point> valid_end_points_;
        int best_global_idx_ = -1;
        int best_traj_idx_ = -1;
        bool reached_ = false;
        bool get_mini_path_ = false;
        std::vector<State> miniPath_;
        
        
        unsigned int inputs_idx_;               //strictly positive

        void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

        void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

        // Used to provide the next input for /drive
        Input GetNextInput();
};  

