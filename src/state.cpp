#include "f110-mpc/state.h"

State::State() : x_(0), y_(0), ori_(0), size_(3)
{
    //ROS_INFO("state created");
}

State::State(double x, double y, double ori): x_(x), y_(y), ori_(ori), size_(3)
{   
    //x,y,ori populated
    //ROS_INFO("state created");
}

State::~State()
{
    //ROS_INFO("killing the state");
}

Eigen::VectorXd State::StateToVector()
{
    Eigen::VectorXd state_vector;
    state_vector.resize(size_);
    state_vector << x_, y_ , ori_;
    return state_vector;
}

//Setters
void State::set_x(double x)
{
    x_ = x;
}

void State::set_y(double y)
{
    y_ = y;
}

void State::set_ori(double ori)
{
    ori_ = ori;
}

std::pair<float,float> State::GetPair()
{
    return std::pair<float,float>(x_,y_);
}

// Getters
double State::x()
{
    return x_;
}

double State::y()
{
    return y_;
}

double State::ori()
{
    return ori_;
}

int State::size()
{
    return size_;
}


