#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <ros/package.h>

#include "input.h"
#include "state.h"
#include "transforms.h"
#include "input.h"
#include "occupancy_grid.h"
#include "visualizer.h"

#include <fstream>
#include <limits>

using namespace std;

class Trajectory
{
    public: 
        Trajectory(ros::NodeHandle &nh);
        ~Trajectory();

        // Converts trajectory of State objects to pairs of X,Y coordinates
        vector<pair<float,float>> GetPairPoints();
        // Loads CSV of waypoints
        bool ReadCSV(string filename);
        //visualize waypoint
        void Visualize();

        ros::Publisher traj_pub_;
        vector<State> waypoints_;
        std::vector<geometry_msgs::Point> points_;
        std::vector<std_msgs::ColorRGBA> colors_;

        std::vector<std::vector<State>> local_dwa_traj_table_;

        void Generate_Table();
    
    private:
        
        float lookahead;

        // Returns distances of waypoints in CSV trajectory relative to
        // car's position
        vector<float> GetWaypointDistances(const geometry_msgs::Pose &pose, bool inFront);
};

#endif