#include "f110-mpc/trajectory.h"

using namespace std;

Trajectory::Trajectory(ros::NodeHandle &nh)
{
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("trajectory", 1);
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
/*
void Trajectory::Generate_Table()
{
    //500 points
    std::vector<State> temp;

    for(int i=1; i <= waypoints_.size() ; i++)
    {
        if(i%50 == 0 && i>=50)
        {   
            local_dwa_traj_table_.push_back(temp);
        }
        temp.push_back(waypoints_.at(i-1));
    }
}
*/