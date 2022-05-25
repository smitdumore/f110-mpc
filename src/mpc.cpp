#include "f110-mpc/mpc.h" 

MPC::MPC(ros::NodeHandle &nh): input_size_(2) , state_size_(3) , constraints_(nh) , nh_(nh)
{
    nh.getParam("/horizon", N_);
    nh.getParam("/dt", dt_);

    //double desired_vel, desired_steer, q0, q1, q2, r0, r1;

    //nh.getParam("des_vel", desired_vel);
    //nh.getParam("des_steer", desired_steer);
    nh_.getParam("/q0", q0);
    nh_.getParam("/q1", q1);
    nh_.getParam("/q2", q2);
    nh_.getParam("/r0", r0);
    nh_.getParam("/r1", r1);

    //desired_input_.set_v(desired_vel);
    //desired_input_.set_steer_ang(desired_steer);
    Eigen::DiagonalMatrix<double, 3> q;
    Eigen::DiagonalMatrix<double, 2> r;
    q.diagonal() << q0, q1, q2;
    r.diagonal() << r0, r1;
    cost_ = Cost(q, r);

    nx_ = state_size_ =  3;
    nu_ = input_size_ = 2;
    
    
    num_inputs_=(input_size_ * N_);
    num_states_=(state_size_ * (N_ + 1));
    num_variables_=(num_states_ + num_inputs_);
    num_constraints_=(num_states_ + 2 * (N_ + 1) + num_inputs_); // dynamics + follow the gap + max/min input 

    /*
    lower_bound_.resize(num_constraints_);
    upper_bound_.resize(num_constraints_);

    CreateHessianMatrix();
    CreateLinearConstraintMatrix();
    CreateUpperBound();
    CreateLowerBound();

    QPsolution_ = Eigen::VectorXd::Zero(num_variables_);
    */

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
    return N_;
}

void MPC::UpdateScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    scan_msg_ = *scan_msg;
}

void MPC::initMPC(std::vector<State> &ref_state_trajectory, std::vector<Input> &ref_inputs, State current_state)
{
    current_state_ = current_state;
    //desired_state_trajectory_ = desired_state_trajectory;
    //model_.Linearize(current_state_ , input , dt_);
    constraints_set_state(current_state_);
    constraints_.FindHalfSpaces(current_state_ , scan_msg_);

    Eigen::SparseMatrix<double> H_matrix((N_+1)*(nx_+nu_), (N_+1)*(nx_+nu_));
    Eigen::SparseMatrix<double> A_c((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_, (N_+1)*(nx_+nu_));

    // define the gradient vector
    Eigen::VectorXd g((N_+1)*(nx_+nu_));
    g.setZero();

    // the upper and lower bound constraint vectors
    Eigen::VectorXd lb((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_);
    Eigen::VectorXd ub((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_);

    // define the matrices (vectors) for state and control references
    // at each time step
    Eigen::Matrix<double, nx_, 1> x_ref;
    Eigen::Matrix<double, nu_, 1> u_ref;

    // define the matrices for discrete dynamics
    Eigen::Matrix<double, nx_, 1> hd;
    Eigen::Matrix<double, nx_, nx_> Ad;
    Eigen::Matrix<double, nx_, nu_> Bd;

    /** MAIN LOOP **/
    for (int i=0; i<N_+1; i++)
    {
        x_ref = ref_state_trajectory[i];      //both are vector but ??
        u_ref = ref_input[i];
        getCarDynamics(Ad, Bd, hd, x_ref, u_ref);
 
        // fill the H_matrix with state cost Q for the first (N+1)*nx
        // diagonal and input cost R along the next (N+1)*nu diagonal
        if (i > 0)
        {
            for (int row=0; row<nx_; row++)
            {
                H_matrix.insert(i*nx_ + row, i*nx_ + row) = cost_.q(row, row);
            }
 
            for (int row=0; row<nu_; row++)
            {
                H_matrix.insert(((N_+1)*nx_) + (i*nu_+row), ((N_+1)*nx_) + (i*nu_+row)) = cost_.r(row, row);
            }
 
                g.segment<nx_>(i*nx_) << -(cost_.q)*x_ref;
                g.segment<nu_>(((N_+1)*nx_) + i*nu_) << -(cost_.r)*u_ref;
        }
 
            // fill the constraint matrix first with the dynamic constraint
            // x_k+1 = Ad*x_k + Bd*u_k + hd
        if (i < N_)
        {
                for (int row=0; row<nx_; row++)
                {
                    for (int col=0; col<nx_; col++)
                    {
                            A_c.insert((i+1)*nx_ + row, i*nx_ + col) = Ad(row, col);
                    }
                }
 
                for (int row=0; row<nx_; row++)
                {
                    for (int col=0; col<nu_; col++)
                    {
                        A_c.insert((i+1)*nx_ + row, (N_+1)*nx_ + i*nu_ + col) = Bd(row, col);
                    }
                }
 
                lb.segment<nx_>((i+1)*nx_) = -hd;
                ub.segment<nx_>((i+1)*nx_) = -hd;
        }
 
            for (int row=0; row<nx_; row++)
            {
                A_c.insert(i*nx_+row, i*nx_+row)  = -1.0;
            }
 
            // fill Ax <= B
            A_c.insert(((N_+1)*nx_) + 2*i, (i*nx_))= constraints_.l1()(0);
            A_c.insert(((N_+1)*nx_) + 2*i, (i*nx_)+1) = constraints_.l1()(1);;
 
            A_c.insert(((N_+1)*nx_) + 2*i+1, (i*nx_)) = constraints_.l2()(0);;
            A_c.insert(((N_+1)*nx_) + 2*i+1, (i*nx_)+1) = constraints_.l2()(1);;
 
            lb(((N_+1)*nx_) + 2*i) = -OsqpEigen::INFTY;
            ub(((N_+1)*nx_) + 2*i) = constraints_.l1()(2);
 
            lb(((N_+1)*nx_) + 2*i+1) = -OsqpEigen::INFTY;
            ub(((N_+1)*nx_) + 2*i+1) = constraints_.l2()(2);
 
            // fill u_min < u < u_max in A_c
            for(int row=0; row<nu_; row++)
            {
                A_c.insert((N_+1)*nx_+2*(N_+1)+i*nu_+row, (N_+1)*nx_+i*nu_+row) = 1.0;
            }
 
            lb((N_+1)*nx_ + 2*(N_+1) + i*nu_) = 0.0;
            ub((N_+1)*nx_ + 2*(N_+1) + i*nu_) = 4.5;              //parametrized this
 
            lb((N_+1)*nx_ + 2*(N_+1) + i*nu_ + 1) = -0.43f;
            ub((N_+1)*nx_ + 2*(N_+1) + i*nu_ + 1) = 0.43f;
    }// for ends


    // fill initial condition in lb and ub
    lb.head(nx_) = -ref_trajectory[0];
    ub.head(nx_) = -ref_trajectory[0];

    // ?? 
    lb((N_+1)*nx_ + 2*(N_+1)) = current_ego_vel_;
    ub((N_+1)*nx_ + 2*(N_+1)) = current_ego_vel_;
 
    Eigen::SparseMatrix<double> H_matrix_T = H_matrix.transpose();
    Eigen::SparseMatrix<double> sparse_I((N_+1)*(nx_+nu_), (N_+1)*(nx_+nu_));
    sparse_I.setIdentity();
 
    H_matrix = 0.5*(H_matrix + H_matrix_T) + 0.0000001*sparse_I;
 
    // osqp Eigen solver from https://robotology.github.io/osqp-eigen/doxygen/doc/html/index.html
    // instantiate the solver
    OsqpEigen::Solver solver;
 
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables((N_+1)*(nx_+nu_));
    solver.data()->setNumberOfConstraints((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_);
 
    if(!solver.data()->setHessianMatrix(H_matrix)) throw "failed to set Hessian";
    if(!solver.data()->setGradient(g)) throw "failed to set gradient";
    if(!solver.data()->setLinearConstraintsMatrix(A_c)) throw "failed to set constraint matrix";
    if(!solver.data()->setLowerBound(lb)) throw "failed to set lower bound";
    if(!solver.data()->setUpperBound(ub)) throw "failed to set upper bound";
 
    if(!solver.initSolver()) throw "failed to initialize solver";
 
    planner::Inputs mpc_input;
    mpc_input.header.stamp = ros::Time::now();
    mpc_input.header.frame_id = ego_base_frame_;
 
    if(!solver.solve())
    {
        ROS_ERROR("SOLVER FAILED _____");
        return;
    }
 
    Eigen::VectorXd QPSolution = solver.getSolution();
 
    //visualizeMPC(QPSolution);
 
    const auto start_idx = (N_+1)*nx_;
 
    for (int i=start_idx; i<QPSolution.size(); i+=2)
    {
        mpc_input.speed.push_back(QPSolution(i));
        mpc_input.steering.push_back(QPSolution(i+1));
    }
           
 
    solver.clearSolver();
    
}

void MPC::getCarDynamics(Eigen::Matrix<double,nx_,nx_>& Ad, Eigen::Matrix<double,nx_,nu_>& Bd, Eigen::Matrix<double,nx_,1>& hd, Eigen::Matrix<double,nx_,1>& state, Eigen::Matrix<double,nu_,1>& input)
{
    double yaw = state(2);
            double v = input(0);
            double steer = input(1);

            Eigen::VectorXd dynamics(state.size());
            dynamics(0) = input(0)*cos(state(2));
            dynamics(1) = input(0)*sin(state(2));
            dynamics(2) = tan(input(1))*input(0)/C_l_;

            Eigen::Matrix<double,nx_,nx_> Ak, M12;
            Eigen::Matrix<double,nx_,nu_> Bk;

            Ak << 0.0, 0.0, (-v*sin(yaw)), 0.0, 0.0, (v*cos(yaw)), 0.0, 0.0, 0.0;
            Bk << cos(yaw), 0.0, sin(yaw), 0.0, tan(steer)/C_l_, v/(cos(steer)*cos(steer)*C_l_);

            // from document: https://www.diva-portal.org/smash/get/diva2:1241535/FULLTEXT01.pdf, page 50
            Eigen::Matrix<double,nx_+nx_,nx_+nx_> aux, M;
            aux.setZero();
            aux.block<nx_,nx_>(0,0) << Ak;
            aux.block<nx_,nx_>(0,nx_) << Eigen::Matrix3d::Identity();
            M = (aux*Ts_).exp();
            M12 = M.block<nx_,nx_>(0,nx_);

            Eigen::VectorXd hc(3);
            hc = dynamics - (Ak*state + Bk*input);

            // Discretize
            Ad = (Ak*Ts_).exp();
            Bd = M12*Bk;
            hd = M12*hc;
}
