#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "rosgraph_msgs/Clock.h"
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <eigen3/Eigen/Geometry>
#include "trajectoryGeneration.hpp"

class ArmsInducedOscillation {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber linkStateSub_;
        ros::Subscriber gazeboClockSub_;
        ros::Publisher j1APosPub_, j2APosPub_, j3APosPub_, j4APosPub_, j5APosPub_, j6APosPub_, \
                       j1BPosPub_, j2BPosPub_, j3BPosPub_, j4BPosPub_, j5BPosPub_, j6BPosPub_;
        ros::ServiceClient setModelConfigurationClient_;
        gazebo_msgs::SetModelConfiguration configSrv_;
        double platformLinkPose_[7];
        double gazeboTime_;
        double resetWaitTime_;
    public:
        ArmsInducedOscillation();
        void run();
        void generateOscillation();
        void linkStateCB(gazebo_msgs::LinkStates);          // Link state callback
        void gazeboClockCB(rosgraph_msgs::Clock);           // Gazebo clock callback
};


