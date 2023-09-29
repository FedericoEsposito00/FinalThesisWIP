#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/WrenchStamped.h"

#define RATE 100

using namespace std;

class SENSOR_FILTER {
	public:
		SENSOR_FILTER();
		void cb(geometry_msgs::WrenchStamped::ConstPtr msg);
		void pub_filtered();
	private:
		double _fx;
		double _fy;
		double _fz;
		double _tx;
		double _ty;
		double _tz;
		double _filtered_fx;
		double _filtered_fy;
		double _filtered_fz;
		double _filtered_tx;
		double _filtered_ty;
		double _filtered_tz;
		bool _active;

		ros::NodeHandle _nh;
		ros::Publisher _topic_pub;
		ros::Subscriber _topic_sub;
		ros::Rate _rate;
};

SENSOR_FILTER::SENSOR_FILTER(): _rate(RATE) {
	_topic_sub = _nh.subscribe("/licasa1/sensor_wall_joint_sensor", 1, &SENSOR_FILTER::cb, this);
	_topic_pub = _nh.advertise<geometry_msgs::WrenchStamped> ("/licasa1/sensor_wall_joint_sensor_filtered", 1);
	_fx = _fy = _fz = _tx = _ty = _tz = 0;
	_filtered_fx = _filtered_fy = _filtered_fz = _filtered_tx = _filtered_ty = _filtered_tz = 0;
	_active = false;

	boost::thread(&SENSOR_FILTER::pub_filtered, this);
}

//Callback function: the input of the function is the data to read
void SENSOR_FILTER::cb(geometry_msgs::WrenchStamped::ConstPtr msg) {
	_fx = msg->wrench.force.x;
	_fy = msg->wrench.force.y;
	_fz = msg->wrench.force.z;
	_tx = msg->wrench.torque.x;
	_ty = msg->wrench.torque.y;
	_tz = msg->wrench.torque.z;

	// _fx = 1;
	// _fy = 1;
	// _fz = 1;
	// _tx = 1;
	// _ty = 1;
	// _tz = 1;

	if (!_active) {
		_active = true;
	}
	//ROS_INFO("I heard: _x_speed = %f, _y_speed = %f, _z_speed = %f\n", _x_speed, _y_speed, _z_speed);
}

void SENSOR_FILTER::pub_filtered() {
	//ROS_INFO("First time in pub_filtered\n");

	_filtered_fx = _fx;
	_filtered_fy = _fy;
	_filtered_fz = _fz;
	_filtered_tx = _tx;
	_filtered_ty = _ty;
	_filtered_tz = _tz;

	while (_active) {
		sleep(1);
	}

	geometry_msgs::WrenchStamped filtered;
	filtered.header.frame_id = "sensor_wall";

	while (ros::ok()) {
		_filtered_fx = 0.9*_filtered_fx + 0.1*_fx;
		_filtered_fy = 0.9*_filtered_fy + 0.1*_fy;
		_filtered_fz = 0.9*_filtered_fz + 0.1*_fz;
		_filtered_tx = 0.9*_filtered_tx + 0.1*_tx;
		_filtered_ty = 0.9*_filtered_ty + 0.1*_ty;
		_filtered_tz = 0.9*_filtered_tz + 0.1*_tz;

		filtered.wrench.force.x = _filtered_fx;
		filtered.wrench.force.y = _filtered_fy;
		filtered.wrench.force.z = _filtered_fz;
		filtered.wrench.torque.x = _filtered_tx;
		filtered.wrench.torque.y = _filtered_ty;
		filtered.wrench.torque.z = _filtered_tz;

		filtered.header.stamp = ros::Time::now(),

		_topic_pub.publish(filtered);

		_rate.sleep();
	}
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "SensorFilter");
	ROS_INFO("SENSOR FILTER STARTED\n");

	//Create the sensor filter class object
	SENSOR_FILTER sf;
	ROS_INFO("CONSTRUCTOR WORKED\n");

	ros::spin();

	return 0;
}
