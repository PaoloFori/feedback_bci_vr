#include <ros/ros.h>
#include "feedback_bci/Training.h"


int main(int argc, char** argv) {

    // ros initialization
    ros::init(argc, argv, "training_node");

    feedback::Training training;

    if(training.configure() == false) {
        ROS_ERROR("Training configuration failed");
        ros::shutdown();
        return 0;
    }


    training.run();

    ros::shutdown();

    return 0;
    
    
}
