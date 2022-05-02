#include "f110-mpc/model.h"
const double CAR_LENGTH = 0.35;
Model::Model()
{
    // ROS_INFO("model created");
}

Model::~Model()
{
    // ROS_INFO("killing the model");
}

Eigen::MatrixXd Model::A()
{
    return A_;
}


Eigen::MatrixXd Model::B()
{
    return B_;
}


Eigen::MatrixXd Model::C()
{
    return C_;
}

void Model::Linearize(State &S, Input &I, double dt)
{
    float L = 0.3302f;
    // a_.resize(3,3);
    A_ = Eigen::MatrixXd::Zero(3, 3);

    // b_.resize(3,2);
    B_ = Eigen::MatrixXd::Zero(3, 2);

    // c_.resize(3,1);
    C_ = Eigen::MatrixXd::Zero(3, 1);

    A_(0,2) = -1*I.v()*sin(S.ori())*dt;
    A_(1,2) = I.v()*cos(S.ori())*dt;
    A_(0,0) = 1;
    A_(1,1) = 1;
    A_(2,2) = 1;

    B_(0,0) = cos(S.ori())*dt;
    B_(1,0) = sin(S.ori())*dt;
    B_(2,0) = tan(I.steer_ang())*dt/L;
    B_(2,1) = I.v()*pow(cos(I.steer_ang()),-2)*dt/L;

    C_(0,0) = I.v()*S.ori()*sin(S.ori())*dt;
    C_(1,0) = -1*I.v()*S.ori()*cos(S.ori())*dt;
    C_(2,0) = -1*I.steer_ang()*I.v()*pow(cos(I.steer_ang()),-2)*dt/L;


    //how is this linear ?? 
}

void Model::simulate_dynamics(State& state, Input &input, double dt, State& new_state)
{
    Eigen::VectorXd dynamics(state.size());
    Eigen::VectorXd state_vector;
    Eigen::VectorXd new_state_vector;

    dynamics(0) = input.v() * cos(state.ori());
    dynamics(1) = input.v() * sin(state.ori());
    dynamics(2) = tan(input.steer_ang()) * input.v()/CAR_LENGTH;
    state_vector = state.StateToVector();
    new_state_vector = state_vector + dynamics*dt;
    
    new_state.set_x(new_state_vector(0));
    new_state.set_y(new_state_vector(1));
    new_state.set_ori(new_state_vector(2));
}