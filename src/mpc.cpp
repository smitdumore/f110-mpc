#include "f110-mpc/mpc.h" 

MPC::MPC(ros::NodeHandle &nh): input_size_(2) , state_size_(3) , constraints_(nh) , nh_(nh)
{
    nh.getParam("/horizon", horizon_);
    nh.getParam("/dt", dt_);

    double desired_vel, desired_steer, q0, q1, q2, r0, r1;

    nh.getParam("des_vel", desired_vel);
    nh.getParam("des_steer", desired_steer);
    nh_.getParam("/q0", q0);
    nh_.getParam("/q1", q1);
    nh_.getParam("/q2", q2);
    nh_.getParam("/r0", r0);
    nh_.getParam("/r1", r1);

    desired_input_.set_v(desired_vel);
    desired_input_.set_steer_ang(desired_steer);
    Eigen::DiagonalMatrix<double, 3> q;
    Eigen::DiagonalMatrix<double, 2> r;
    q.diagonal() << q0, q1, q2;
    r.diagonal() << r0, r1;
    cost_ = Cost(q, r);
    
    num_inputs_=(input_size_ * horizon_);
    num_states_=(state_size_ * (horizon_ + 1));
    num_variables_=(num_states_ + num_inputs_);
    num_constraints_=(num_states_ + 2 * (horizon_ + 1) + num_inputs_); // dynamics + follow the gap + max/min input 

    hessian_.resize(num_variables_, num_variables_);
    gradient_.resize(num_variables_);
    linear_matrix_.resize(num_constraints_, num_variables_);
    lower_bound_.resize(num_constraints_);
    upper_bound_.resize(num_constraints_);

    CreateHessianMatrix();
    CreateLinearConstraintMatrix();
    CreateUpperBound();
    CreateLowerBound();

    QPsolution_ = Eigen::VectorXd::Zero(num_variables_);

    mpc_pub_ = nh.advertise<visualization_msgs::Marker>("mpc", 1);
    ROS_INFO("mpc created");
    
}

MPC::~MPC()
{
    ROS_INFO("killing the mpc");
}

float MPC::dt()
{
    return dt_;
}

int MPC::horizon()
{
    return horizon_;
}

void MPC::UpdateScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    scan_msg_ = *scan_msg;
}

void MPC::Update(State current_state , Input input, std::vector<State> &desired_state_trajectory)
{
    current_state_ = current_state;
    desired_state_trajectory_ = desired_state_trajectory;
    model_.Linearize(current_state_ , input , dt_);
    constraints_set_state(current_state_);
    constrints_.FindHalfSpaces(current_state_ , scan_msg_);
}