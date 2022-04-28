#include "f110-mpc/trajectory.h"

using namespace std;

Trajectory::Trajectory()
{
}

Trajectory::~Trajectory()
{

}


bool Trajectory::ReadCSV(string filename)
{
    string path = ros::package::getPath("f110-mpc")+"/csv/"+filename+".csv";
    cout << path << endl;
    ros::Duration(10.0).sleep();
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