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


class UnforcedOscillation {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber linkStateSub_;
        ros::Subscriber gazeboClockSub_;
        ros::ServiceClient setModelConfigurationClient_;
        gazebo_msgs::SetModelConfiguration configSrv_;
        double platformLinkPose_[7];
        double gazeboTime_;
        double resetWaitTime_;
    public:
        UnforcedOscillation();
        void run();
        void generateOscillation();
        void linkStateCB(gazebo_msgs::LinkStates);          // Link state callback
        void gazeboClockCB(rosgraph_msgs::Clock);           // Gazebo clock callback
};


// Constructor
UnforcedOscillation::UnforcedOscillation() {
    setModelConfigurationClient_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");        // Set Model Configuration Client
    linkStateSub_ = nh_.subscribe("/gazebo/link_states",1000,&UnforcedOscillation::linkStateCB,this);                                    // Link State subscriber
    gazeboClockSub_ = nh_.subscribe("/clock",0,&UnforcedOscillation::gazeboClockCB,this);                                        // Gazebo Clock Subscriber

    // Definition of the service request message
    configSrv_.request.model_name = "CraneBot";                             // Model name
    configSrv_.request.urdf_param_name = "robot_description";               // URDF param name (unused)

    // Definition of the joint names and initial position vectors
    std::vector<std::string> joints_name_vec;                               // Joint names [rad] vector
    std::vector<double> joints_pos_vec;                                     // Joint position [rad] vector
    joints_name_vec.insert(joints_name_vec.end(), {"cables_joint_z","cables_joint_x","cables_joint_y","platform_joint_x","platform_erb145_joint_A","platform_erb145_joint_B"});
    joints_pos_vec.insert(joints_pos_vec.end(), {0.0, 0.0, 0.08, 0.0, 0, 0});      // [rad]

    // Configuration of the service
    configSrv_.request.joint_names = joints_name_vec;                       // Configuring the server with joints name list
    configSrv_.request.joint_positions = joints_pos_vec;

    // Initializations
    gazeboTime_ = 0;                                                        // Initializing Gazebo time variable
    resetWaitTime_ = 5;                                                     // Time to wait while positioning the system to a nonrest configuration [sec]
}


// Callback for saving the pose of the links
void UnforcedOscillation::linkStateCB(gazebo_msgs::LinkStates lstate) {
    platformLinkPose_[0] = lstate.pose[6].position.x;                     // Position
    platformLinkPose_[1] = lstate.pose[6].position.y;
    platformLinkPose_[2] = lstate.pose[6].position.z;
    platformLinkPose_[3] = lstate.pose[6].orientation.x;                  // Orientation in quaternions
    platformLinkPose_[4] = lstate.pose[6].orientation.y;
    platformLinkPose_[5] = lstate.pose[6].orientation.z;
    platformLinkPose_[6] = lstate.pose[6].orientation.w;
}


// Callback for saving time from gazebo. Conversion in second
void UnforcedOscillation::gazeboClockCB(rosgraph_msgs::Clock clockMsg) {
    double sec = clockMsg.clock.sec;
    double nsec = clockMsg.clock.nsec;
    double num = 1000000000;
    gazeboTime_ = sec+nsec/num;
}


// Modify the initial state of the system
void UnforcedOscillation::generateOscillation() {

    // Define Sampling rate
    ros::Rate rate(50);

    // Initialization for time sampling
    double startTime = gazeboTime_;
    double elapsedTime = 0; 
    double currentTime = 0;
    double oldElapsed = 0;

    // Move the robot to a non-rest initial configuration by calling the gazebo/setModelConfiguration service and waiting resetWaitTime_ seconds
    std::cout << "Moving the robot to a non-rest configuration" << std::endl;
    for (int i=0; i<2; i++) {
        while (elapsedTime<=oldElapsed+resetWaitTime_) {
            currentTime = gazeboTime_; 
            elapsedTime = currentTime - startTime;
            setModelConfigurationClient_.call(configSrv_);          // Set model initial state
            rate.sleep();
        }
        oldElapsed = elapsedTime;    
    }
    std::cout << "Starting pose successfully modified" << std::endl << "Unforced oscillation generated" << std::endl;
}


// Run loop
void UnforcedOscillation::run() {
    std::thread generateOscillation_t(&UnforcedOscillation::generateOscillation,this);
    ros::spin();
}


// Main loop
int main (int argc, char** argv) {
    ros::init(argc, argv, "UnforcedOscillation");
    UnforcedOscillation unforcedOscillationGenerator;
    ros::Duration(0.5).sleep(); // sleep for half a second
    unforcedOscillationGenerator.run();
    return 0;
}