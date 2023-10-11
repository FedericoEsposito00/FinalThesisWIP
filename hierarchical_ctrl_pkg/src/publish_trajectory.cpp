#include "ros/ros.h"
#include "boost/thread.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <ros/package.h>
#include <fstream>
#include <string>
#include <iostream>

#define RATE 100
#define Kp 1e-2
#define Ki 1e-2

using namespace std;


class PUB_TRA {
	public:
		PUB_TRA();
		void cb(std_msgs::Float64MultiArray::ConstPtr msg);
		void control_cb(std_msgs::Float64MultiArray::ConstPtr msg);
		void pub_trajectory();
	private:

		int N;
		
		double* P_x;
		double* P_y;
		double* P_z;
		double* P_psi;
		double* P_d_x;
		double* P_d_y;
		double* P_d_z;
		double* P_d_psi;
		double* P_dd_x;
		double* P_dd_y;
		double* P_dd_z;
		double* P_dd_psi;

		double est[6];

		bool activate_control;
		double F_des; // absolute value of the desired force
		double angle; // angle of the force that the robot feels from the environment

		ros::NodeHandle _nh;
		ros::Publisher _topic_pub;
		ros::Subscriber _topic_sub;
		ros::Subscriber _activate_control_sub;
		ros::Rate _rate;
};

PUB_TRA::PUB_TRA(): _rate(RATE) {
	_topic_sub = _nh.subscribe("/licasa1/estimate", 1, &PUB_TRA::cb, this);
	_topic_pub = _nh.advertise< std_msgs::Float64MultiArray > ("/licasa1/ref_topic", 1);
	_activate_control_sub = _nh.subscribe("/licasa1/activate_control", 1, &PUB_TRA::control_cb, this);

	ifstream ref;

	string pkg_loc = ros::package::getPath("hierarchical_ctrl_pkg");
	ref.open(pkg_loc + "/UAV.txt", ios::in); 

	if (!ref.is_open()) {
		cout<<"Error opening the file!";
		exit(1);
	}

	// trajectory length
	ref>>N;

	P_x = new double[N];
	P_y = new double[N];
	P_z = new double[N];
    P_psi = new double[N];
	P_d_x = new double[N];
	P_d_y = new double[N];
	P_d_z = new double[N];
    P_d_psi = new double[N];
	P_dd_x = new double[N];
	P_dd_y = new double[N];
	P_dd_z = new double[N];
    P_dd_psi = new double[N];

	for (int i = 0; i<N; i++) {
		ref>>P_x[i];
		ref>>P_y[i];
		ref>>P_z[i];
        ref>>P_psi[i];
		ref>>P_d_x[i];
		ref>>P_d_y[i];
		ref>>P_d_z[i];
        ref>>P_d_psi[i];
		ref>>P_dd_x[i];
		ref>>P_dd_y[i];
		ref>>P_dd_z[i];
        ref>>P_dd_psi[i];
	}

	for (int i = 0; i < 6; i++) {
		est[i] = 0;
	}

	F_des = 0;
	angle = 0;

	activate_control = false;

	ref.close();
	
	boost::thread(&PUB_TRA::pub_trajectory, this);
}

//Callback function: the input of the function is the data to read
void PUB_TRA::cb(std_msgs::Float64MultiArray::ConstPtr msg) {
	for (int i = 0; i < 6; i++) {
		est[i] = msg->data[i];
	}
}

//Callback function: the input of the function is the data to read
void PUB_TRA::control_cb(std_msgs::Float64MultiArray::ConstPtr msg) {
	if (msg->data[0] > 0) {
		activate_control = true;
	} else {
		activate_control = false;
	}

	F_des = msg->data[1];
	angle = msg->data[2];
}

void PUB_TRA::pub_trajectory() {

	int ii = 0;
	std_msgs::Float64MultiArray ref_msg;

	double err_x = 0;
	double err_x_int = 0;
	double delta_x = 0;
	double delta_x_send = 0;
	double err_y = 0;
	double err_y_int = 0;
	double delta_y = 0;
	double delta_y_send = 0;

	while (ros::ok()) {

		if (activate_control) {
			err_x = F_des*cos(angle) - est[0];
			err_x_int = err_x_int + err_x*1/RATE;;
			delta_x = -Kp*err_x - Ki*err_x_int;
			delta_x_send = 0.9*delta_x_send + 0.1*delta_x;
			err_y = F_des*sin(angle) - est[1];
			err_y_int = err_y_int + err_y*1/RATE;;
			delta_y = -Kp*err_y - Ki*err_y_int;
			delta_y_send = 0.9*delta_y_send + 0.1*delta_y;
			cout<<"delta_x: "<<delta_x<<endl;
			cout<<"delta_x_send: "<<delta_x_send<<endl;
			cout<<"delta_y: "<<delta_y<<endl;
			cout<<"delta_y_send: "<<delta_y_send<<endl;
		} else {
			err_x = 0;
			err_x_int = 0;
			delta_x = 0;
			if (abs(delta_x_send) < 1e-4) {
				delta_x_send = 0;
			} else {
				delta_x_send = 0.995*delta_x_send;
			}
			err_y = 0;
			err_y_int = 0;
			delta_y = 0;
			if (abs(delta_y_send) < 1e-4) {
				delta_y_send = 0;
			} else {
				delta_y_send = 0.995*delta_y_send;
			}
			cout<<"delta_y_send: "<<delta_y_send<<endl;
			cout<<"delta_x_send: "<<delta_x_send<<endl;
		}

		ref_msg.data.clear();
		ref_msg.data.push_back(P_x[ii] + delta_x_send*cos(P_psi[ii]) - delta_y_send*sin(P_psi[ii]));
		//cout<<"x: "<<P_x[ii] + delta_x_send*cos(P_psi[ii]) - delta_y_send*sin(P_psi[ii])<<endl;
		ref_msg.data.push_back(P_y[ii] + delta_x_send*sin(P_psi[ii]) + delta_y_send*cos(P_psi[ii]));
		//cout<<"y: "<<P_y[ii] + delta_x_send*sin(P_psi[ii]) + delta_y_send*cos(P_psi[ii])<<endl;
		ref_msg.data.push_back(P_z[ii]);
		ref_msg.data.push_back(P_psi[ii]);
		ref_msg.data.push_back(P_d_x[ii]);
		ref_msg.data.push_back(P_d_y[ii]);
		ref_msg.data.push_back(P_d_z[ii]);
		ref_msg.data.push_back(P_d_psi[ii]);
		ref_msg.data.push_back(P_dd_x[ii]);
		ref_msg.data.push_back(P_dd_y[ii]);
		ref_msg.data.push_back(P_dd_z[ii]);
		ref_msg.data.push_back(P_dd_psi[ii]);

		_topic_pub.publish(ref_msg);

		// cout<<"REF PUBLISHED\n";

		if (ii < N-1) {
			ii++;
		}

		_rate.sleep();
		
	}
}

int main( int argc, char** argv ) {

	//Init the ros node with name
	ros::init(argc, argv, "PublishTrajectory");
	ROS_INFO("TRAJECTORY PUBLISHER STARTED\n");

	PUB_TRA pb;
	
	ros::spin();

	return 0;
}
