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

void JOY_WRAP::pub_ref() {
	//ROS_INFO("First time in pub_ref\n");
	double x = _ref_trans.getOrigin().x();
	double y = _ref_trans.getOrigin().y();
	double z = _ref_trans.getOrigin().z();
	tf::TransformListener listener;
	tf::StampedTransform left_eef;
	tf::StampedTransform right_eef;
	while (!listener.waitForTransform("shoulder_link_y", "left_eef_link", ros::Time(0), ros::Duration(1)) || !listener.waitForTransform("world", "right_eef_link", ros::Time(0), ros::Duration(1))) {
		sleep(1);
	}
	while (ros::ok()) {
		listener.lookupTransform("shoulder_link_y","left_eef_link",ros::Time(0), left_eef);
		listener.lookupTransform("shoulder_link_y","right_eef_link",ros::Time(0), right_eef);

		x = (left_eef.getOrigin().x()+right_eef.getOrigin().x())/2;
		y = (left_eef.getOrigin().y()+right_eef.getOrigin().y())/2;
		z = (left_eef.getOrigin().z()+right_eef.getOrigin().z())/2;
		x = x + _x_speed/RATE_CTRL;
		y = y + _y_speed/RATE_CTRL;
		z = z + _z_speed/RATE_CTRL;

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
	ros::init(argc, argv, "VelJoyWrap");
	ROS_INFO("JOYWRAP STARTED\n");

	//Create the ROS_SUB class object
	JOY_WRAP wrap;

	ros::spin();

	return 0;
}
