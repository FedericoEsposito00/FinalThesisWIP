#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "rosgraph_msgs/Clock.h"
#include <string>
#include <iostream>
#include "boost/thread.hpp"
#include <chrono>
#include <fstream>

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "mav_msgs/Actuators.h"
#include <std_msgs/Float64.h>
#include <ros/package.h>

//Include Tf libraries
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

//Include Simulink-generated library
#include "hc/HierarchicalControl.h"   

#define RATE 100

using namespace std;

class HC_NODE {
    public:
        HC_NODE();

        void ctrl_loop();
        void run();

    private:
        ros::NodeHandle _nh;

        //Simulink-generated object for inverse kinematics
        HierarchicalControl rtObj;

        //state variables
        double xd[3]; //
        double xd_old[3]; //
        double xd_dot[3]; //
        double xd_dot_old[3]; //
        double xd_ddot[3]; //
        double psi_d;

        double x_curr[3]; //
        double x_old[3]; //

        double x_dot[3]; //

        double eta_curr[3];
        double eta_old[3];

        double eta_dot[3]; //

        //ROS topic objects
		ros::Publisher _motor_speed_pub;

        //TF objects
        tf::TransformListener _listener;
        tf::StampedTransform _tf_body;

		tf::TransformBroadcaster _trans_br;

};

HC_NODE::HC_NODE() {

    _motor_speed_pub = _nh.advertise< mav_msgs::Actuators > ("/licasa1/command/motor_speed", 1);
	
    while (!_listener.waitForTransform("NED_world", "licasa1/NED_frame", ros::Time(0), ros::Duration(1))){
		sleep(1);
        cout<<"Waiting for world transform\n";
	}

    _listener.lookupTransform("NED_world","licasa1/NED_frame",ros::Time(0), _tf_body);

    xd[0] = xd_old[0] = x_curr[0] = x_old[0] = _tf_body.getOrigin().x();
    xd[1] = xd_old[1] = x_curr[1] = x_old[1] = _tf_body.getOrigin().y();
    xd[2] = xd_old[2] = x_curr[2] = x_old[2] = _tf_body.getOrigin().z();

    x_dot[0] = x_dot[1] = x_dot[2] = xd_dot[0] = xd_dot[1] = xd_dot[2] = xd_dot_old[0] = xd_dot_old[1] = xd_dot_old[2] = xd_ddot[0] = xd_ddot[1] = xd_ddot[2]= 0;

    double roll, pitch, yaw;

    tf::Matrix3x3 m( _tf_body.getRotation() ); // quaternion to RPY
	m.getRPY(roll, pitch, yaw);

    eta_curr[0] = eta_old[0] = roll;
    eta_curr[1] = eta_old[1] = pitch;
    eta_curr[2] = eta_old[2] = psi_d = yaw;
    
    rtObj.initialize();

    for (int i = 0; i < 3; i++) {
        rtObj.rtU.position[i] = x_curr[i];
        rtObj.rtU.linear_vel[i] = x_dot[i];
        rtObj.rtU.eta[i] = eta_curr[i];
        rtObj.rtU.eta_dot[i] = eta_dot[i];
        rtObj.rtU.position_des[i] = xd[i];
        rtObj.rtU.vel_linear_des[i] = xd_dot[i];
        rtObj.rtU.acc_linear_des[i] = xd_ddot[i];
    }

    rtObj.rtU.psi_d = psi_d;

}

void HC_NODE::ctrl_loop() {
    double roll, pitch, yaw;

    // while (!_listener.waitForTransform("NED_world", "UAV_des_frame", ros::Time(0), ros::Duration(1))){
	// 	sleep(1);
    //     cout<<"Waiting for goal\n";
	// }
    //cout<<"goal found\n";

    //Lock the code to start manually the execution of the trajectory
	// cout << "Press enter to start the trajectory execution" << endl;
	// string ln;
	// getline(cin, ln);

	ifstream ref;

	string pkg_loc = ros::package::getPath("hierarchical_ctrl_pkg");
	ref.open(pkg_loc + "/UAV.txt", ios::in); 

	if (!ref.is_open()) {
		cout<<"Error opening the file!";
		exit(1);
	}

	//trajectory length
	int N;
	ref>>N;

	double* P_x = new double[N];
	double* P_y = new double[N];
	double* P_z = new double[N];
    double* P_psi = new double[N];
	double* P_d_x = new double[N];
	double* P_d_y = new double[N];
	double* P_d_z = new double[N];
    double* P_d_psi = new double[N];
	double* P_dd_x = new double[N];
	double* P_dd_y = new double[N];
	double* P_dd_z = new double[N];
    double* P_dd_psi = new double[N];

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

	ref.close();

    //Index for the trajectory vectors
    int ii = 0;

    ros::Rate r(RATE);

    mav_msgs::Actuators cmd;

    cmd.angular_velocities.clear();

    for (int i = 0; i < 4; i++) {
        cmd.angular_velocities.push_back(2620);
    }

    cout<<"OPEN LOOP START\n";

    _motor_speed_pub.publish(cmd);

    sleep(5);

    cout<<"OPEN LOOP END\n";

    while(ros::ok()) {

        _listener.lookupTransform("NED_world","licasa1/NED_frame",ros::Time(0), _tf_body);

        x_curr[0] = _tf_body.getOrigin().x();
        x_curr[1] = _tf_body.getOrigin().y();
        x_curr[2] = _tf_body.getOrigin().z();

        //cout<<"x_curr read\n";

        tf::Matrix3x3 m1( _tf_body.getRotation() ); // quaternion to RPY
        m1.getRPY(roll, pitch, yaw);

        eta_curr[0] = roll;
        eta_curr[1] = pitch;
        eta_curr[2] = yaw;

        //cout<<"eta_curr read\n";

        // _listener.lookupTransform("NED_world","UAV_des_frame",ros::Time(0), _tf_body);
        // tf::Matrix3x3 m2( _tf_body.getRotation() ); // quaternion to RPY
        // m2.getRPY(roll, pitch, yaw);

        // xd[0] = _tf_body.getOrigin().x();
        // xd[1] = _tf_body.getOrigin().y();
        // xd[2] = _tf_body.getOrigin().z();
        // psi_d = yaw;

        xd[0] = P_x[ii];
        xd[1] = P_y[ii];
        xd[2] = P_z[ii];
        psi_d = P_psi[ii];
        xd_dot[0] = P_d_x[ii];
        xd_dot[1] = P_d_y[ii];
        xd_dot[2] = P_d_z[ii];
        xd_ddot[0] = P_dd_x[ii];
        xd_ddot[1] = P_dd_y[ii];
        xd_ddot[2] = P_dd_z[ii];

        //cout<<"goal position and orientation read\n";

        for (int i = 0; i < 3; i++) {
            x_dot[i] = (x_curr[i]-x_old[i])/RATE;
            eta_dot[i] = (eta_curr[i]-eta_old[i])/RATE;
            x_old[i] = x_curr[i];
            eta_old[i] = eta_curr[i];
        }

        //cout<<"derivatives computed\n";

        for (int i = 0; i < 3; i++) {
            rtObj.rtU.position[i] = x_curr[i];
            rtObj.rtU.linear_vel[i] = x_dot[i];
            rtObj.rtU.eta[i] = eta_curr[i];
            rtObj.rtU.eta_dot[i] = eta_dot[i];
            rtObj.rtU.position_des[i] = xd[i];
            rtObj.rtU.vel_linear_des[i] = xd_dot[i];
            rtObj.rtU.acc_linear_des[i] = xd_ddot[i];
            // cout<<"position "<<i<<": "<<x_curr[i]<<endl;
            // cout<<"lin_vel "<<i<<": "<<x_dot[i]<<endl;
            // cout<<"eta "<<i<<": "<<eta_curr[i]<<endl;
            // cout<<"eta_dot "<<i<<": "<<eta_dot[i]<<endl;
            // cout<<"position des "<<i<<": "<<xd[i]<<endl;
            // cout<<"lin_vel_des "<<i<<": "<<xd_dot[i]<<endl;
            // cout<<"lin_acc_des "<<i<<": "<<xd_ddot[i]<<endl;
        }

        rtObj.rtU.psi_d = psi_d;

        // cout<<"psi_d: "<<psi_d<<endl;

        rtObj.step();

        //cout<<"rtObj step called\n";

		mav_msgs::Actuators cmd;

        cmd.angular_velocities.clear();

        for (int i = 0; i < 4; i++) {
		    cmd.angular_velocities.push_back(rtObj.rtY.velocities[i]);
            // cout<<"rotor speed "<<i<<": "<<rtObj.rtY.velocities[i]<<endl;
        }

        cout<<"u: \n";
        for (int i = 0; i<6; i++) {
            cout<<rtObj.rtY.u[i]<<endl;
        }

        // cout<<"mu_d: \n";
        // for (int i = 0; i<3; i++) {
        //     cout<<rtObj.rtY.mu_d[i]<<endl;
        // }

        // cout<<"estimate: \n";
        // for (int i = 0; i<6; i++) {
        //     cout<<rtObj.rtY.estimate[i]<<endl;
        // }

        cout<<"Q: \n";
        for (int i = 0; i<3; i++) {
            cout<<rtObj.rtY.Q[3*i]<<" "<<rtObj.rtY.Q[3*i+1]<<" "<<rtObj.rtY.Q[3*i+2]<<endl;
        }

		//Publish all the commands in topics
	    _motor_speed_pub.publish(cmd);

        cout<<"speeds published\n\n";

        if (ii < N-1) {
            ii++;
        }

		r.sleep();
	}

}

void HC_NODE::run() {
    boost::thread( &HC_NODE::ctrl_loop, this);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "sensors_hc_ctrl");
	HC_NODE hcn;
	cout<<"Constructor worked\n";

    hcn.run();

    ros::spin();

	return 0;
}


