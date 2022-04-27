#ifndef INPUT_H
#define INPUT_H


#include <ros/ros.h>
#include <Eigen/Geometry>

// Specifies the format for the Input object/vector of velocity and steering angle
// input vector is vel , steering ang

class Input
{
    public:
        Input();                    //default constructor
        Input(double v, double steer_ang);
        //additional constructor for size ??

        virtual ~Input();

        Eigen::VectorXd InputToVector();
        
        //setters
        void set_v(double v);
        void set_steer_ang(double steer_ang);

        //getters
        double v();
        double steer_ang();

    private:
        double v_;
        double steer_ang_;
        int size_; //size of input vector
};

#endif