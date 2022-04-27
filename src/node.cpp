#include <ros/ros.h>
#include "f110-mpc/project.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    project project_obj(nh);
    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
