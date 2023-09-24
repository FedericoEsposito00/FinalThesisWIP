#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "boost/thread.hpp"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/JointState.h"

//Include KDL libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#define RATE_CTRL 100
#define L_half 0.18
#define NJ 4
#define pi 3.14159

#define right_q_min0 -1.570
#define right_q_max0 1.570
#define right_q_min1 -1.570
#define right_q_max1 0.349
#define right_q_min2 -1.570
#define right_q_max2 1.570
#define right_q_min3 -2.617
#define right_q_max3 2.617
#define left_q_min0 -1.570
#define left_q_max0 1.570
#define left_q_min1 -0.349
#define left_q_max1 1.570
#define left_q_min2 -1.570
#define left_q_max2 1.570
#define left_q_min3 -2.617
#define left_q_max3 2.617

using namespace std;

class JOY_WRAP {
	public:
		JOY_WRAP();
		void cb(sensor_msgs::Joy::ConstPtr msg);
		void joint_states_cb( sensor_msgs::JointState js );
		void pub_ref();
	private:
		double _rescaleValue;
		double _active; // References are only computed while pressing button[7] (R2)
		double _override; // Override boundaries if pressing button[6] (L2)
		double _x_speed;
		double _y_speed;
		double _z_speed;
		tf::Transform _ref_trans;
		tf::TransformBroadcaster _trans_br;
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Rate _rate;

		//Kinematic chain
		KDL::Chain _right_k_chain;
		KDL::Chain _left_k_chain;
		//Kinematic tree
		KDL::Tree _k_tree;

		// KDL::ChainFkSolverPos_recursive *_right_fksolver; 
		// KDL::ChainFkSolverPos_recursive *_left_fksolver; 

		// KDL::ChainIkSolverVel_pinv *_right_ik_solver_vel;
		// KDL::ChainIkSolverVel_pinv *_left_ik_solver_vel;

		KDL::ChainIkSolverPos_LMA *_right_ik_solver_pos;
		KDL::ChainIkSolverPos_LMA *_left_ik_solver_pos;

		//Variable to store the joint configuration
		KDL::JntArray *_right_q_in;
		KDL::JntArray *_left_q_in;

		KDL::Frame F_dest;
};

JOY_WRAP::JOY_WRAP(): _rate(RATE_CTRL) {
	_rescaleValue = 0.1;
	_x_speed = 0;
	_y_speed = 0;
	_z_speed = 0;
	_topic_sub = _nh.subscribe("/joy", 1, &JOY_WRAP::cb, this);
	tf::TransformListener listener;
	tf::StampedTransform left_eef;
	tf::StampedTransform right_eef;
	while (!listener.waitForTransform("shoulder_link_y", "left_eef_link", ros::Time(0), ros::Duration(1)) || !listener.waitForTransform("world", "right_eef_link", ros::Time(0), ros::Duration(1))) {
		sleep(1);
	}
	listener.lookupTransform("shoulder_link_y","left_eef_link",ros::Time(0), left_eef);
	listener.lookupTransform("shoulder_link_y","right_eef_link",ros::Time(0), right_eef);

	double init_x = (left_eef.getOrigin().x()+right_eef.getOrigin().x())/2;
	double init_y = (left_eef.getOrigin().y()+right_eef.getOrigin().y())/2;
	double init_z = (left_eef.getOrigin().z()+right_eef.getOrigin().z())/2;

	_ref_trans.setOrigin(tf::Vector3(init_x, init_y, init_z));

	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	_ref_trans.setRotation(q);

	//Retrieve the robot description (URDF) from the robot_description param
	std::string robot_desc_string;
	_nh.param("/licasa1/robot_description", robot_desc_string, std::string());

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

	_right_ik_solver_pos = new KDL::ChainIkSolverPos_LMA(_right_k_chain, _L);
	_left_ik_solver_pos = new KDL::ChainIkSolverPos_LMA(_left_k_chain, _L);

	_right_q_in = new KDL::JntArray( _right_k_chain.getNrOfJoints() );
	_left_q_in = new KDL::JntArray( _left_k_chain.getNrOfJoints() );

	//The orientation set point is constant
	KDL::Frame ident = KDL::Frame::Identity();
	for(int i=0; i<9; i++ ){
		F_dest.M.data[i] = ident.M.data[i];
	}

	boost::thread(&JOY_WRAP::pub_ref, this);
}

//Callback function: the input of the function is the data to read
void JOY_WRAP::cb(sensor_msgs::Joy::ConstPtr msg) {
	_active = msg->buttons[7];
	_override = msg->buttons[6];
	if (_active) {
		_y_speed = msg->axes[0]*_rescaleValue;
		_z_speed = msg->axes[1]*_rescaleValue;
		_x_speed = msg->axes[3]*_rescaleValue;
	}
	//ROS_INFO("I heard: _x_speed = %f, _y_speed = %f, _z_speed = %f\n", _x_speed, _y_speed, _z_speed);
}

void JOY_WRAP::joint_states_cb( sensor_msgs::JointState js ) {

	//We assume to know the number of joints
	for(int i=0; i<NJ; i++ ) {
		_right_q_in->data[i] = js.position[i+NJ];
		_left_q_in->data[i] = js.position[i];
	}

}

void JOY_WRAP::pub_ref() {
	//ROS_INFO("First time in pub_ref\n");
	double x = _ref_trans.getOrigin().x();
	double y = _ref_trans.getOrigin().y();
	double z = _ref_trans.getOrigin().z();
	while (ros::ok()) {
		x = x + _x_speed/RATE_CTRL;
		y = y + _y_speed/RATE_CTRL;
		z = z + _z_speed/RATE_CTRL;

		int right_err = false;
		int left_err = false;
		KDL::JntArray right_q_out(_right_k_chain.getNrOfJoints());
		KDL::JntArray left_q_out(_right_k_chain.getNrOfJoints());

		F_dest.p.data[0] = x;
		F_dest.p.data[1] = y-L_half;
		F_dest.p.data[2] = z;

		right_err = _right_ik_solver_pos->CartToJnt(*_right_q_in, F_dest, right_q_out);

		// if (right_q_out(0) < right_q_min0 || right_q_out(0) > right_q_max0 || right_q_out(1) < right_q_min1 || right_q_out(1) > right_q_max1 || right_q_out(2) < right_q_min2 || right_q_out(2) > right_q_max2 || right_q_out(3) < right_q_min3 || right_q_out(3) > right_q_max3) {
		// 	right_err = -6;
		// 	cout<<"RIGHT JOINTS RANGE EXCEEDED\n";
		// 	cout<<right_q_out(0)<<endl<<right_q_out(1)<<endl<<right_q_out(2)<<endl<<right_q_out(3)<<endl;
		// }

		if (abs(right_q_out(3)) < 0.05) {
			right_err = -6;
			cout<<"RIGHT SINGULARITY\n";
		}

		F_dest.p.data[1] = y+L_half;

		left_err = _left_ik_solver_pos->CartToJnt(*_left_q_in, F_dest, left_q_out);

		// if (left_q_out(0) < left_q_min0 || left_q_out(0) > left_q_max0 || left_q_out(1) < left_q_min1 || left_q_out(1) > left_q_max1 || left_q_out(2) < left_q_min2 || left_q_out(2) > left_q_max2 || left_q_out(3) < left_q_min3 || left_q_out(3) > left_q_max3) {
		// 	left_err = -6;
		// 	cout<<"LEFT JOINTS RANGE EXCEEDED\n";
		// 	cout<<left_q_out(0)<<endl<<left_q_out(1)<<endl<<left_q_out(2)<<endl<<left_q_out(3)<<endl;
		// }

		if (abs(left_q_out(3)) < 0.05) {
			left_err = -6;
			cout<<"LEFT SINGULARITY\n";
		}

		if((right_err != KDL::SolverI::E_NOERROR || left_err != KDL::SolverI::E_NOERROR) && !_override) {
			x = x - _x_speed/RATE_CTRL;
			y = y - _y_speed/RATE_CTRL;
			z = z - _z_speed/RATE_CTRL;
			ROS_INFO("REFERENCE MOVEMENT PREVENTED\n");
		} 

		tf::Transform _right_ref_trans;
		tf::Transform _left_ref_trans;
		//cout<<"Current ref position computed\n";
		_ref_trans.setOrigin(tf::Vector3(x, y, z));
		_right_ref_trans.setOrigin(tf::Vector3(x, y-L_half, z));
		_left_ref_trans.setOrigin(tf::Vector3(x, y+L_half, z));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		_right_ref_trans.setRotation(q);
		_left_ref_trans.setRotation(q);
		_trans_br.sendTransform(tf::StampedTransform(_ref_trans, ros::Time::now(), "shoulder_link_y", "ref_frame"));
		_trans_br.sendTransform(tf::StampedTransform(_left_ref_trans, ros::Time::now(), "shoulder_link_y", "left_ref_frame"));
		_trans_br.sendTransform(tf::StampedTransform(_right_ref_trans, ros::Time::now(), "shoulder_link_y", "right_ref_frame"));
		//cout<<"Transform sent\n";
		_rate.sleep();
	}
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "BoundedJoyWrap");
	ROS_INFO("JOYWRAP STARTED\n");

	//Create the ROS_SUB class object
	JOY_WRAP wrap;

	ros::spin();

	return 0;
}
