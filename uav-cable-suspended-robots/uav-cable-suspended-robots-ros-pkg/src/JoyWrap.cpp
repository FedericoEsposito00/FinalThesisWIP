#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "boost/thread.hpp"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>

#define RATE 100
#define L_half 0.18

using namespace std;

class JOY_WRAP {
	public:
		JOY_WRAP();
		void cb(sensor_msgs::Joy::ConstPtr msg);
		void activation_cb(std_msgs::Float64::ConstPtr msg);
		void pub_ref();
	private:
		double _rescaleValue;
		double _x_speed;
		double _y_speed;
		double _z_speed;
		int _state; // 0 = shoulder based, 1 = world based
		int _old_state;
		int _button;
		int _old_button;
		bool _active;
		tf::Transform _ref_trans;
		tf::TransformBroadcaster _trans_br;
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Subscriber _activation_sub;
		ros::Rate _rate;
};

JOY_WRAP::JOY_WRAP(): _rate(RATE) {
	_rescaleValue = 0.1;
	_x_speed = 0;
	_y_speed = 0;
	_z_speed = 0;
	_state = _old_state = 0;
	_button = _old_button = 0;
	_active = false;
	_topic_sub = _nh.subscribe("/joy", 1, &JOY_WRAP::cb, this);
	_activation_sub = _nh.subscribe("/licasa1/joywrap_start", 1, &JOY_WRAP::activation_cb, this);
	boost::thread(&JOY_WRAP::pub_ref, this);
}

//Callback function: the input of the function is the data to read
void JOY_WRAP::cb(sensor_msgs::Joy::ConstPtr msg) {
	_y_speed = msg->axes[4]*_rescaleValue; //[0]
	_z_speed = msg->axes[5]*_rescaleValue; //[1]
	_x_speed = msg->axes[3]*_rescaleValue;

	_button = msg->buttons[2];

	if (_button == 1 && _old_button == 0) {
		_state = (_state + 1)%2;
		cout<<"STATE: "<<_state<<endl;
	}
	_old_button = _button;
	//ROS_INFO("I heard: _x_speed = %f, _y_speed = %f, _z_speed = %f\n", _x_speed, _y_speed, _z_speed);
}

void JOY_WRAP::activation_cb(std_msgs::Float64::ConstPtr f) {
	if (f->data > 0 && _active == false) {
		_active = true;
		cout<<"JOY_WRAP activated!\n";
	}
}

void JOY_WRAP::pub_ref() {
	tf::TransformListener listener;
	tf::StampedTransform left_eef;
	tf::StampedTransform right_eef;

	// Wait for the message from CLIK_node that signifies that the initial position has been reached
	cout<<"Waiting until the initial position is reached\n";
	while (_active == false) {
		cout<<_active<<endl;
		sleep(1);
	}
	sleep(5);
	cout<<"JOYWRAP STARTED FROM CLIK\n";
	
	while (!listener.waitForTransform("shoulder_link_y", "left_eef_link", ros::Time(0), ros::Duration(1)) || !listener.waitForTransform("world", "right_eef_link", ros::Time(0), ros::Duration(1))) {
		sleep(1);
		cout<<"WAITING FOR TRANSFORM\n";
	}
	listener.lookupTransform("shoulder_link_y","left_eef_link",ros::Time(0), left_eef);
	listener.lookupTransform("shoulder_link_y","right_eef_link",ros::Time(0), right_eef);

	double x = (left_eef.getOrigin().x()+right_eef.getOrigin().x())/2;
	double y = (left_eef.getOrigin().y()+right_eef.getOrigin().y())/2;
	double z = (left_eef.getOrigin().z()+right_eef.getOrigin().z())/2;

	_ref_trans.setOrigin(tf::Vector3(x, y, z));

	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	_ref_trans.setRotation(q);
	//ROS_INFO("First time in pub_ref\n");
	double x_world = 0;
	double y_world = 0;
	double z_world = 0;
	tf::Transform _right_ref_trans;
	tf::Transform _left_ref_trans;
	while (ros::ok()) {
		if (_state == 1 && _old_state == 0) {
			listener.lookupTransform("world","left_eef_link",ros::Time(0), left_eef);
			listener.lookupTransform("world","right_eef_link",ros::Time(0), right_eef);

			x_world = (left_eef.getOrigin().x()+right_eef.getOrigin().x())/2;
			y_world = (left_eef.getOrigin().y()+right_eef.getOrigin().y())/2;
			z_world = (left_eef.getOrigin().z()+right_eef.getOrigin().z())/2;
		}
		if (_state == 1) {
			x_world = x_world + _x_speed/RATE;
			y_world = y_world + _y_speed/RATE;
			z_world = z_world + _z_speed/RATE;
			//cout<<"Current ref position computed\n";
			_ref_trans.setOrigin(tf::Vector3(x_world, y_world, z_world));
			_right_ref_trans.setOrigin(tf::Vector3(x_world, y_world-L_half, z_world));
			_left_ref_trans.setOrigin(tf::Vector3(x_world, y_world+L_half, z_world));
			q.setRPY(0, 0, 0);
			_right_ref_trans.setRotation(q);
			_left_ref_trans.setRotation(q);
			_trans_br.sendTransform(tf::StampedTransform(_ref_trans, ros::Time::now(), "world", "ref_frame"));
			_trans_br.sendTransform(tf::StampedTransform(_left_ref_trans, ros::Time::now(), "world", "left_ref_frame"));
			_trans_br.sendTransform(tf::StampedTransform(_right_ref_trans, ros::Time::now(), "world", "right_ref_frame"));
		}
		if (_state == 0 && _old_state == 1) {
			listener.lookupTransform("shoulder_link_y","left_eef_link",ros::Time(0), left_eef);
			listener.lookupTransform("shoulder_link_y","right_eef_link",ros::Time(0), right_eef);

			x = (left_eef.getOrigin().x()+right_eef.getOrigin().x())/2;
			y = (left_eef.getOrigin().y()+right_eef.getOrigin().y())/2;
			z = (left_eef.getOrigin().z()+right_eef.getOrigin().z())/2;
		}
		if (_state == 0) {
			x = x + _x_speed/RATE;
			y = y + _y_speed/RATE;
			z = z + _z_speed/RATE;
			//cout<<"Current ref position computed\n";
			_ref_trans.setOrigin(tf::Vector3(x, y, z));
			_right_ref_trans.setOrigin(tf::Vector3(x, y-L_half, z));
			_left_ref_trans.setOrigin(tf::Vector3(x, y+L_half, z));
			q.setRPY(0, 0, 0);
			_right_ref_trans.setRotation(q);
			_left_ref_trans.setRotation(q);
			_trans_br.sendTransform(tf::StampedTransform(_ref_trans, ros::Time::now(), "shoulder_link_y", "ref_frame"));
			_trans_br.sendTransform(tf::StampedTransform(_left_ref_trans, ros::Time::now(), "shoulder_link_y", "left_ref_frame"));
			_trans_br.sendTransform(tf::StampedTransform(_right_ref_trans, ros::Time::now(), "shoulder_link_y", "right_ref_frame"));
		}
		_old_state = _state;
		// cout<<"Active: "<<_active<<endl;
		_rate.sleep();
	}
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "JoyWrap");
	ROS_INFO("JOYWRAP STARTED\n");

	//Create the ROS_SUB class object
	JOY_WRAP wrap;
	ROS_INFO("CONSTRUCTOR WORKED\n");

	ros::spin();

	return 0;
}
