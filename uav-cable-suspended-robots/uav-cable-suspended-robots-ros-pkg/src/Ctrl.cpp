#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "rosgraph_msgs/Clock.h"
#include <string>
#include <iostream>
#include "boost/thread.hpp"
#include <chrono>

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <ros/package.h>

//Include Tf libraries
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h> 

//Include KDL libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#define RATE 100
#define L_half 0.18

#define NJ 4

using namespace std;

class CTRL_NODE {
    public:
        CTRL_NODE();

        void joint_states_cb( sensor_msgs::JointState );
        void goto_initial_position( double dp_l[NJ], double dp_r[NJ] );
        void ctrl_loop();
        void get_dirkin();
        void run();

    private:
        ros::NodeHandle _nh;

        //ROS topic objects
        ros::Subscriber _js_sub;
		ros::Publisher _left_cmd_pub[NJ];
        ros::Publisher _right_cmd_pub[NJ];

        ros::Publisher _right_cartpose_pub;
        ros::Publisher _left_cartpose_pub;

        //TF objects
        tf::TransformListener _listener;
        tf::StampedTransform _tf_ref;

        //Kinematic chain
		KDL::Chain _right_k_chain;
		KDL::Chain _left_k_chain;
		//Kinematic tree
		KDL::Tree _k_tree;

		KDL::ChainIkSolverPos_LMA *_right_ik_solver_pos;
		KDL::ChainIkSolverPos_LMA *_left_ik_solver_pos;

		//Variable to store the joint configuration
		KDL::JntArray *_right_q_in;
		KDL::JntArray *_left_q_in;

		KDL::Frame F_dest;

        KDL::Frame _right_p_out;
        KDL::Frame _left_p_out;

};

CTRL_NODE::CTRL_NODE() {
    _js_sub = _nh.subscribe("/licasa1/joint_states", 0, &CTRL_NODE::joint_states_cb, this);

    _left_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_1_effort_pos_controller/command", 1);
	_left_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_2_effort_pos_controller/command", 1);
	_left_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_3_effort_pos_controller/command", 1);
	_left_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_leftarm_4_effort_pos_controller/command", 1);

    _right_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_1_effort_pos_controller/command", 1);
	_right_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_2_effort_pos_controller/command", 1);
	_right_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_3_effort_pos_controller/command", 1);
	_right_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/licasa1/licasa1_rightarm_4_effort_pos_controller/command", 1);

	//Retrieve the robot description (URDF) from the robot_description param
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());

	//Use the treeFromString function to convert the robot model into a kinematic tree 
	if (!kdl_parser::treeFromString(robot_desc_string, _k_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		exit(1);
	}

	//Define the links of the desired chain base_link -> tip_link
	std::string base_link = "shoulder_link_y";
	std::string right_tip_link  = "right_eef_link";
	std::string left_tip_link  = "left_eef_link";
	if ( !_k_tree.getChain(base_link, right_tip_link, _right_k_chain) ) {
		cout<<"Error getting right chain\n";
		exit(1);
	}
	if ( !_k_tree.getChain(base_link, left_tip_link, _left_k_chain) ) {
		cout<<"Error getting left chain\n";
		exit(1);
	}

	Eigen::Matrix<double, 6, 1 > _L;
	_L(0) = 1;
	_L(1) = 1;
	_L(2) = 1;
	_L(3) = 0;
	_L(4) = 0;
	_L(5) = 0;

	_right_ik_solver_pos = new KDL::ChainIkSolverPos_LMA( _right_k_chain, _L);
	_left_ik_solver_pos = new KDL::ChainIkSolverPos_LMA( _left_k_chain, _L);

	_right_q_in = new KDL::JntArray( _right_k_chain.getNrOfJoints() );
	_left_q_in = new KDL::JntArray( _left_k_chain.getNrOfJoints() );

	//The orientation set point is constant
	KDL::Frame ident = KDL::Frame::Identity();
	for(int i=0; i<9; i++ ){
		F_dest.M.data[i] = ident.M.data[i];
	}

}

//Callback for the joint state
void CTRL_NODE::joint_states_cb( sensor_msgs::JointState js ) {

	//We assume to know the number of joints
    for(int i=0; i<NJ; i++ ) {
		_left_q_in->data[i] = js.position[i];
        _right_q_in->data[i] = js.position[i+NJ];
        //cout<<"I HEARD "<<ql[i]<<" AND "<<qr[i]<<endl;
	}

}

void CTRL_NODE::goto_initial_position( double dp_l[NJ], double dp_r[NJ] ) {
    ros::Rate r(100);

	float min_e = 1000.0;
	float left_max_e = 1000.0;
    float right_max_e = 1000.0;

	std_msgs::Float64 left_cmd[NJ];
    std_msgs::Float64 right_cmd[NJ];

	//While the maximum error over all the left joints is higher than a given threshold 
	while( left_max_e > 0.005 && right_max_e > 0.005) {
 		left_max_e = -1000;
        right_max_e = -1000;
		//Command the same value for all the joints and calculate the maximum error
		for(int i=0; i<NJ; i++) {
 			left_cmd[i].data = dp_l[i];
			_left_cmd_pub[i].publish (left_cmd[i]);
			float left_e = fabs( left_cmd[i].data - _left_q_in->data[i] );
			//max_e is the maximum error over all the joints
			left_max_e = ( left_e > left_max_e ) ? left_e : left_max_e;
            cout<<"Left error: "<<left_max_e<<endl;
        }
        for(int i=0; i<NJ; i++) {
            right_cmd[i].data = dp_r[i];
			_right_cmd_pub[i].publish (right_cmd[i]);
			float right_e = fabs( right_cmd[i].data - _right_q_in->data[i] );
			//max_e is the maximum error over all the joints
			right_max_e = ( right_e > right_max_e ) ? right_e : right_max_e;
            cout<<"Right error: "<<right_max_e<<endl;
		}
		r.sleep();
	}

	sleep(2);
}

void CTRL_NODE::ctrl_loop() {
    ros::Rate r(RATE);

	std_msgs::Float64 left_cmd[NJ];
    std_msgs::Float64 right_cmd[NJ];

    //Control the robot towards a fixed initial position
	double left_i_cmd[NJ];
	left_i_cmd[0] = 0;
	left_i_cmd[1] = 0;
	left_i_cmd[2] = 0;
	left_i_cmd[3] = -1.5;

    double right_i_cmd[NJ];
	right_i_cmd[0] = 0;
	right_i_cmd[1] = 0;
	right_i_cmd[2] = 0;
	right_i_cmd[3] = -1.5;

    //Lock the code to start manually the execution of the trajectory
	cout << "Press enter to start the trajectory execution" << endl;
	string ln;
	getline(cin, ln);

    goto_initial_position(left_i_cmd, right_i_cmd);
    cout<<"Initial position reached\n";

    while (!_listener.waitForTransform("shoulder_link_y", "ref_frame", ros::Time(0), ros::Duration(1))){
		sleep(1);
        cout<<"Waiting for reference\n";
	}

    while(ros::ok()) {

        _listener.lookupTransform("shoulder_link_y","ref_frame",ros::Time(0), _tf_ref);

        KDL::JntArray q_out(_right_k_chain.getNrOfJoints());

        F_dest.p.data[0] = _tf_ref.getOrigin().x();
        F_dest.p.data[1] = _tf_ref.getOrigin().y()-L_half;
        F_dest.p.data[2] = _tf_ref.getOrigin().z();

        _right_ik_solver_pos->CartToJnt(*_right_q_in, F_dest, q_out);

        for (int i =0; i<NJ; i++) {
            right_cmd[i].data = q_out.data[i];
        }

        F_dest.p.data[0] = _tf_ref.getOrigin().x();
        F_dest.p.data[1] = _tf_ref.getOrigin().y()+L_half;
        F_dest.p.data[2] = _tf_ref.getOrigin().z();

        _left_ik_solver_pos->CartToJnt(*_left_q_in, F_dest, q_out);

        for (int i =0; i<NJ; i++) {
            left_cmd[i].data = q_out.data[i];
        }

		//Publish all the commands in topics
		for(int i=0; i<NJ; i++) {
			_left_cmd_pub[i].publish(left_cmd[i]);
            _right_cmd_pub[i].publish(right_cmd[i]);
		}

		r.sleep();
	}

}

void CTRL_NODE::run() {
    boost::thread( &CTRL_NODE::ctrl_loop, this);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "kdl_ctrl");
	CTRL_NODE cn;
	cout<<"Constructor worked\n";

    cn.run();

    ros::spin();

	return 0;
}


