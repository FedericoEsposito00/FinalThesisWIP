#include "ArmsInducedOscillationCranebot.hpp"

// Constructor
ArmsInducedOscillation::ArmsInducedOscillation() {                                         // Unpause Gazebo Client
    setModelConfigurationClient_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");        // Set Model Configuration Client
    linkStateSub_ = nh_.subscribe("/gazebo/link_states",0,&ArmsInducedOscillation::linkStateCB,this);                                       // Link State subscriber
    gazeboClockSub_ = nh_.subscribe("/clock",0,&ArmsInducedOscillation::gazeboClockCB,this);                                                // Gazebo Clock Subscriber
    j1APosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/platform_erb145_joint_A_effort_pos_controller/command", 0);		    // Publisher adversisers
    j2APosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/erb145_link2_joint_A_effort_pos_controller/command", 0);
    j3APosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/link2_erb145_joint_A_effort_pos_controller/command", 0);
    j4APosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/erb145_link4_joint_A_effort_pos_controller/command", 0);
    j5APosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/link4_erb115_joint_A_effort_pos_controller/command", 0);
    j6APosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/link5_gripper_joint_A_effort_pos_controller/command", 0);
    j1BPosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/platform_erb145_joint_B_effort_pos_controller/command", 0);		    // Publisher adversisers
    j2BPosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/erb145_link2_joint_B_effort_pos_controller/command", 0);
    j3BPosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/link2_erb145_joint_B_effort_pos_controller/command", 0);
    j4BPosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/erb145_link4_joint_B_effort_pos_controller/command", 0);
    j5BPosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/link4_erb115_joint_B_effort_pos_controller/command", 0);
    j6BPosPub_ = nh_.advertise<std_msgs::Float64>("/cranebot/link5_gripper_joint_B_effort_pos_controller/command", 0);

    // Definition of the service request message
    configSrv_.request.model_name = "CraneBot";                             // Model name
    configSrv_.request.urdf_param_name = "robot_description";               // URDF param name (unused)

    // Definition of the joint names and initial position vectors
    std::vector<std::string> joints_name_vec;                               // Joint names [rad] vector
    std::vector<double> joints_pos_vec;                                     // Joint position [rad] vector
    joints_name_vec.insert(joints_name_vec.end(), {"cables_joint_z","cables_joint_x","cables_joint_y","platform_erb145_joint_A","platform_erb145_joint_B"});
    joints_pos_vec.insert(joints_pos_vec.end(), {0.0, 0.0, 0.0, 0.0, 0.0});      // [rad]

    // Configuration of the service
    configSrv_.request.joint_names = joints_name_vec;                       // Configuring the server with joints name list
    configSrv_.request.joint_positions = joints_pos_vec;

    // Initializations
    gazeboTime_ = 0;                                                        // Initializing Gazebo time variable
    resetWaitTime_ = 5;                                                     // Time to wait while positioning the system to a nonrest configuration [sec]
}


// Callback for saving the pose of the links
void ArmsInducedOscillation::linkStateCB(gazebo_msgs::LinkStates lstate) {
    platformLinkPose_[0] = lstate.pose[6].position.x;                     // Position
    platformLinkPose_[1] = lstate.pose[6].position.y;
    platformLinkPose_[2] = lstate.pose[6].position.z;
    platformLinkPose_[3] = lstate.pose[6].orientation.x;                  // Orientation in quaternions
    platformLinkPose_[4] = lstate.pose[6].orientation.y;
    platformLinkPose_[5] = lstate.pose[6].orientation.z;
    platformLinkPose_[6] = lstate.pose[6].orientation.w;
}


// Callback for saving time from gazebo. Conversion in second
void ArmsInducedOscillation::gazeboClockCB(rosgraph_msgs::Clock clockMsg) {
    double sec = clockMsg.clock.sec;
    double nsec = clockMsg.clock.nsec;
    double num = 1000000000;
    gazeboTime_ = sec+nsec/num;
}


void ArmsInducedOscillation::generateOscillation() {

    ros::Rate r(1000);

    double dt = 0.001;
    std::vector<double> tDes, qDes, qDotDes, qDDotDes, qDes2, qDotDes2, qDDotDes2, qDes3, qDotDes3, qDDotDes3;

    std::vector<double> sDotInit{0,0,0,0,0,0};
    std::vector<double> sDotFinal{0,0,0,0,0,0};

    std::vector<double> tFinalA{2,2,2,2,2,2};
    std::vector<double> tFinalB{1.5,1.5,1.5,1.5,1.5,1.5};
    std::vector<double> tFinal;

    std::vector<double> sInit1{0,0,0,0,0,0};
    std::vector<double> sInit2{0,0,0,0,0,0};

    std::vector<std::vector<double>> t1, s1, sdot1, sdotdot1;   
    std::vector<std::vector<double>> t2, s2, sdot2, sdotdot2;


    // Arms induced oscillation around Z axis (Comment, compile and restart, to see X axis oscillations)
    double amp = 0.15;
    tFinal = tFinalB;
    std::vector<double> sFinal1_1{0,-amp,0,0,0,0};
    std::vector<double> sFinal1_2{0,amp,0,0,0,0};
    std::vector<double> sFinal1_3{0,-amp,0,0,0,0};
    std::vector<double> sFinal1_4{0,amp,0,0,0,0};
    std::vector<double> sFinal1_5{0,-amp,0,0,0,0};
    std::vector<double> sFinal1_6{0,amp,0,0,0,0};
    std::vector<double> sFinal1_7{0,-amp,0,0,0,0};
    std::vector<double> sFinal1_8{0,amp,0,0,0,0};
    std::vector<double> sFinal1_9{0,-amp,0,0,0,0};
    std::vector<double> sFinal1_10{0,amp,0,0,0,0};
    std::vector<double> sFinal1_11{0,0,0,0,0,0};

    std::vector<double> sFinal2_1{0,-amp,-0,0,0,0};
    std::vector<double> sFinal2_2{0,amp,0,0,0,0};
    std::vector<double> sFinal2_3{0,-amp,-0,0,0,0};
    std::vector<double> sFinal2_4{0,amp,0,0,0,0};
    std::vector<double> sFinal2_5{0,-amp,-0,0,0,0};
    std::vector<double> sFinal2_6{0,amp,0,0,0,0};
    std::vector<double> sFinal2_7{0,-amp,-0,0,0,0};
    std::vector<double> sFinal2_8{0,amp,0,0,0,0};
    std::vector<double> sFinal2_9{0,-amp,-0,0,0,0};
    std::vector<double> sFinal2_10{0,amp,0,0,0,0};
    std::vector<double> sFinal2_11{0,0,0,0,0,0};

    // Arms induced oscillation around X axis (Uncomment, compile and restart, to see X axis oscillations)
    // double amp = 0.3;
    // tFinal = tFinalA;
    // std::vector<double> sFinal1_1{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal1_2{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal1_3{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal1_4{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal1_5{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal1_6{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal1_7{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal1_8{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal1_9{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal1_10{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal1_11{0,0,0,0,0,0};

    // std::vector<double> sFinal2_1{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal2_2{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal2_3{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal2_4{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal2_5{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal2_6{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal2_7{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal2_8{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal2_9{0,-amp,amp,0,0,0};
    // std::vector<double> sFinal2_10{0,amp,-amp,0,0,0};
    // std::vector<double> sFinal2_11{0,0,0,0,0,0};

    multiJointCubicVelTraj(dt, tFinal, sInit1, sFinal1_1, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sInit2, sFinal2_1, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_1, sFinal1_2, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_1, sFinal2_2, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_2, sFinal1_3, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_2, sFinal2_3, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_3, sFinal1_4, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_3, sFinal2_4, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_4, sFinal1_5, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_4, sFinal2_5, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_5, sFinal1_6, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_5, sFinal2_6, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_6, sFinal1_7, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_6, sFinal2_7, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_7, sFinal1_8, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_7, sFinal2_8, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_8, sFinal1_9, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_8, sFinal2_9, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_9, sFinal1_10, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_9, sFinal2_10, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);
    multiJointCubicVelTraj(dt, tFinal, sFinal1_10, sFinal1_11, sDotInit, sDotFinal, s1, sdot1, sdotdot1, t1);
    multiJointCubicVelTraj(dt, tFinal, sFinal2_10, sFinal2_11, sDotInit, sDotFinal, s2, sdot2, sdotdot2, t2);


    for (int i=0; i<20000; i++) {
        s1[1].push_back(s1[1][s1[1].size()-1]);
        sdot1[1].push_back(sdot1[1][sdot1[1].size()-1]);
        s2[1].push_back(s2[1][s2[1].size()-1]);
        sdot2[1].push_back(sdot2[1][sdot2[1].size()-1]);

        s1[2].push_back(s1[2][s1[2].size()-1]);
        sdot1[2].push_back(sdot1[2][sdot1[2].size()-1]);
        s2[2].push_back(s2[2][s2[2].size()-1]);
        sdot2[2].push_back(sdot2[2][sdot2[2].size()-1]);
    }


    std_msgs::Float64 j2ACommand, j2BCommand, j3ACommand, j3BCommand;
    std::vector<double> OrientationZ;
    std::vector<double> timeVec;

    double start_time = gazeboTime_;
    double elapsed_time;

    for (int i = 0; i<s1[1].size(); i++) {

        j2ACommand.data = s1[1][i];
        j2BCommand.data = s2[1][i];
        j3ACommand.data = s1[2][i];
        j3BCommand.data = s2[2][i];
        j2APosPub_.publish(j2ACommand);
        j2BPosPub_.publish(j2BCommand);
        j3APosPub_.publish(j3ACommand);
        j3BPosPub_.publish(j3BCommand);

        r.sleep();

    }
}



// Run loop
void ArmsInducedOscillation::run() {
    std::thread generateOscillation_t(&ArmsInducedOscillation::generateOscillation,this);
    ros::spin();
}


// Main loop
int main (int argc, char** argv) {
    ros::init(argc, argv, "armsInducedOscillation");
    ArmsInducedOscillation armsInducedOscillation;
    ros::Duration(0.5).sleep(); // sleep for half a second
    armsInducedOscillation.run();
    return 0;
}