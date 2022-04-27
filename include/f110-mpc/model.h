#ifndef MODEL_H
#define MODEL_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include "state.h"
#include "input.h"

// Linearizes the dynamics given the current state and its input using
// Forward Euler discretization of the kinematic model. Accurate upto dt=200ms.
// Equation: x_(t+1) = Ax_(t)+Bu(t)+C

class Model
{
    public:
        Model();            //default constructor

        virtual ~Model();

        Eigen::MatrixXd A();
        Eigen::MatrixXd B();
        Eigen::MatrixXd C();

        void Linearize(State &S, Input &I, double dt);

    private:
        double time_step_;
        Eigen::MatrixXd A_;
        Eigen::MatrixXd B_;
        Eigen::MatrixXd C_;
};

#endif