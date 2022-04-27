#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <Eigen/Geometry>

//state of the car is 
// x,y,oreint

class State{

    public:
        State();    //default constructor
        State(double x, double y, double ori);

        virtual ~State();
        /***
         * Deleting a derived class object using a pointer of base class type
         *  that has a non-virtual destructor results in undefined behavior. 
         *  To correct this situation, 
         *  the base class should be defined with a virtual destructor
         ***/

        //convert state to an Eigen vector
        Eigen::VectorXd StateToVector();

        //setters
        void set_x(double x);
        void set_y(double y);
        void set_ori(double ori);

        std::pair<float, float> GetPair();

        //getters
        double x();
        double y();
        double ori();
        int size();

    private:
        double x_;
        double y_;
        double ori_;
        int size_;            //size of state vector
};

#endif