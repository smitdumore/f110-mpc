#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "transforms.h"
#include "input.h"
#include "state.h"
#include "transforms.h"
#include "input.h"
#include "occupancy_grid.h"
#include <ros/package.h>

#include <limits>

using namespace std;

class Trajectory
{
    public: 
        Trajectory(float lookahead);
        ~Trajectory();

        // Converts trajectory of State objects to pairs of X,Y coordinates
        vector<pair<float,float>> GetPairPoints();
        // Loads CSV of waypoints
        bool ReadCSV(string filename);
    
    private:
        vector<State> waypoints_;
        float lookahead;

        // Returns distances of waypoints in CSV trajectory relative to
        // car's position
        vector<float> GetWaypointDistances(const geometry_msgs::Pose &pose, bool inFront);
        
};