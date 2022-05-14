#include "f110-mpc/trajectory.h"

using namespace std;

Trajectory::Trajectory(ros::NodeHandle &nh)
{
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("trajectory", 1);
    global_point_pub_ = nh.advertise<visualization_msgs::Marker>("global_point", 10);

    nh.getParam("/lookahead", lookahead);
}

Trajectory::~Trajectory()
{
    //ROS_INFO("Killing trajectory");
}

bool Trajectory::ReadCSV(string filename)
{
    string path = ros::package::getPath("f110-mpc")+"/csv/"+filename+".csv";
    cout << path << endl;
    
    ifstream input(path);
    string coordX, coordY;
    vector<pair<float,float>> temp;
    if (input.is_open())
    {
        while (getline(input,coordX,','))
        {
            getline(input,coordY);
            temp.push_back(pair<float,float>(stof(coordX),stof(coordY)));
        }
    }
    else
    {
        cout << "Please run this from the root catkin_ws directory" << endl;
        return false;
    }
    waypoints_.clear();
    for (unsigned int i = 0; i < temp.size(); i++)
    {
        float prev_x = temp[(i-1)%temp.size()].first;
        float prev_y = temp[(i-1)%temp.size()].second;
        float x = temp[i].first;
        float y = temp[i].second;
        float ori = atan2(y-prev_y,x-prev_x);
        State state;
        state.set_x(x);
        state.set_y(y);
        state.set_ori(ori);

        waypoints_.push_back(state);      //stl vector of state
    }
    return true;
}

void Trajectory::Visualize()      //visualises the global path
{
    std::vector<pair<float,float>> traj;
    
    for (int i = 0; i< waypoints_.size(); i++)
    {
        std::pair<float,float> p;
        p.first = waypoints_.at(i).x();
        p.second = waypoints_.at(i).y();
        traj.push_back(p);
    }

    std::vector<geometry_msgs::Point> traj_points = Visualizer::GenerateVizPoints(traj);
    std::vector<std_msgs::ColorRGBA> traj_colors = Visualizer::GenerateVizColors(traj, 1, 0, 0);

    points_.insert(points_.end(), traj_points.begin(), traj_points.end());
    colors_.insert(colors_.end(), traj_colors.begin(), traj_colors.end());

    traj_pub_.publish( Visualizer::GenerateList(points_, colors_) );
      
    points_.clear();
    colors_.clear();
}

int Trajectory::get_best_global_idx(geometry_msgs::Pose current_pose)
{
    //csv points are in map frame
    //current pose is also in map frame
    
    //need to transform points in base link to check front or back

    float minDistance = numeric_limits<float>::max();
    int closest_idx = -1;
    
    geometry_msgs::TransformStamped world_to_base = transforms_.WorldToCarTransform(current_pose);

    for(int i=0; i < waypoints_.size(); i++)
    {   
        pair<float, float> point;
        point.first = waypoints_.at(i).x();
        point.second = waypoints_.at(i).y();
        pair<float, float> transformedPoint = transforms_.TransformPoint(point, world_to_base);
        
        if(transformedPoint.first < 0){continue;} // behind base link // x is nagative
        double distance = pow( pow(transformedPoint.first,2) + pow(transformedPoint.second,2), 0.5);
        double lookahead_diff = std::abs(distance - lookahead);
        if(lookahead_diff < minDistance)
        {
                minDistance = lookahead_diff;
                closest_idx = i;
        }
    }

    visualization_msgs::Marker global_marker;
    global_marker.id = 1;
    global_marker.header.frame_id = "map";
    global_marker.type = visualization_msgs::Marker::SPHERE;
    global_marker.scale.x = global_marker.scale.y = global_marker.scale.z = 0.2;
    global_marker.color.b = 1.0;
    global_marker.color.g = global_marker.color.r = 0.0;
    global_marker.color.a = 1.0;

    global_marker.pose.position.x = waypoints_.at(closest_idx).x();
    global_marker.pose.position.y = waypoints_.at(closest_idx).y();
    global_marker.pose.position.z = 0;

    global_point_pub_.publish(global_marker);       //publish best follow point

    return closest_idx;
}



