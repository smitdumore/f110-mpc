#ifndef MPC_H
#define MPC_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <unsupported/Eigen/MatrixFunctions>


#include "constraints.h"
#include "state.h"
#include "model.h"
#include "cost.h"
//#include "visualizer.h"

const int nx_ = 3;
const int nu_ = 2;

class MPC
{
    public:
        MPC();
        MPC(ros::NodeHandle &nh);
        virtual ~MPC();

        // Runs one iteration of MPC given the latest state from callback and last MPC input
        // Uses desired_state_trajectory for tracking reference
        void initMPC(std::vector<State> ref_state_trajectory, std::vector<Input> ref_inputs);

        // Generates pretty lines
        void Visualize();
        // Updates scan_msg content
        void UpdateScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

        // accessor functions
        Constraints constraints();
        float dt();                 //initialising variable values to 1
        int horizon();
        std::vector<Input> solved_inputs_;
        std::vector<Input> solved_trajectory();

    private:
        int N_;  // horizon
        // const int nx_ = 3;
        // const int nu_ = 2;
        int input_size_;
        int state_size_;
        int num_states_;
        int num_inputs_;
        int num_variables_;
        int num_constraints_;
        bool solver_init_ = false;
        float dt_;

        Eigen::SparseMatrix<double> hessian_;
        Eigen::VectorXd gradient_;
        Eigen::SparseMatrix<double> linear_matrix_;      //??
        Eigen::VectorXd lower_bound_;
        Eigen::VectorXd upper_bound_;

        Constraints constraints_;
        Model model_;
        State current_state_;
        Input desired_input_;
        std::vector<State> desired_state_trajectory_;
        Cost cost_;
        
        sensor_msgs::LaserScan scan_msg_;
        Eigen::VectorXd QPsolution_;
        OsqpEigen::Solver solver_;
        //std::vector<Input> solved_trajectory_;             //is this the trajectory returned my MPC solver ??

        
        ros::NodeHandle nh_;
        ros::Publisher mpc_pub_;
        
        void getCarDynamics(Eigen::Matrix<double,nx_,nx_>& Ad, Eigen::Matrix<double,nx_,nu_>& Bd, Eigen::Matrix<double,nx_,1>& hd, Eigen::Matrix<double,nx_,1>& state, Eigen::Matrix<double,nu_,1>& input);

};

#endif