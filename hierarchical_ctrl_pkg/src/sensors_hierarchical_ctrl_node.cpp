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
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"

//Include Tf libraries
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

//Include Simulink-generated library
#include "hc/HierarchicalControl.h"   
#include "est/Estimator.h"

#define RATE 100

using namespace std;

class HC_NODE {
    public:
        HC_NODE();

        void odom_callback(nav_msgs::Odometry odom);
		void imu_callback(sensor_msgs::Imu imu);

        void ctrl_loop();
        void run();

    private:
        ros::NodeHandle _nh;

        // Simulink-generated object for hierarchical controller
        HierarchicalControl rtObj;

        // Simulnk-generated object for estimator
        Estimator est;

        // reference variables
        Eigen::Vector3d _pos_ref;
		Eigen::Vector3d	_pos_ref_dot;
		Eigen::Vector3d	_pos_ref_dot_dot;
        double psi_d;
        double dot_psi_d;
        double ddot_psi_d;

        // current state variables
        Eigen::Vector3d _omega_b_b;		//angular velocity expressed in body NED frame
		Eigen::Vector3d _p_b;			//position expressed in world NED frame
		Eigen::Vector3d _p_b_dot;		//velocity expressed in world NED frame
		Eigen::Vector3d _eta_b;			//RPY angles 
		Eigen::Vector3d _eta_b_dot;		//RPY angles derivatives

        // variables used in the sensor callbacks
        Eigen::Matrix3d _Rb;			//expresses body orientation in world frame
		Eigen::Matrix3d _R_enu2ned;		//to transform from ENU to NED frames
        Eigen::Matrix3d _Q;				//same definitions as seen in lectures
		Eigen::Matrix3d _Q_dot;
		Eigen::Matrix3d _Ib;
		Eigen::Matrix3d _C;	
		Eigen::Matrix3d _M;

        // control flags
        bool _first_odom;
        bool _first_imu;

        // Controller and Estimator outputs
        double estimate[6];
        double u[3];
        double tau[3];

        // ROS topic objects
		ros::Publisher _motor_speed_pub;
        ros::Subscriber _odom_sub;
        ros::Subscriber _imu_sub;

        // TF objects
        tf::TransformListener _listener;
        tf::StampedTransform _tf_body;

};

HC_NODE::HC_NODE() {

    // ROS topic initialization
    _motor_speed_pub = _nh.advertise< mav_msgs::Actuators > ("/licasa1/command/motor_speed", 1);

    _odom_sub = _nh.subscribe("/licasa1/ground_truth/odometry", 0, &HC_NODE::odom_callback, this);	
	_imu_sub = _nh.subscribe("/licasa1/ground_truth/imu", 0, &HC_NODE::imu_callback, this);	

    // array initialization
    _pos_ref(0) = _pos_ref(1) = _pos_ref(2) = 0;
    _pos_ref_dot(0) = _pos_ref_dot(1) = _pos_ref_dot(2) = 0;
    _pos_ref_dot_dot(0) = _pos_ref_dot_dot(1) = _pos_ref_dot_dot(2) = 0;
    psi_d = dot_psi_d = ddot_psi_d = 0;

    _omega_b_b(0) = _omega_b_b(1) = _omega_b_b(2) = 0;

    _p_b(0) = _p_b(1) = _p_b(2) = 0;
    _p_b_dot(0) = _p_b_dot(1) = _p_b_dot(2) = 0;

    _eta_b(0) = _eta_b(1) = _eta_b(2) = 0;
    _eta_b_dot(0)= _eta_b_dot(1) = _eta_b_dot(2) = 0;

    // matrix initialization
    _Rb.setIdentity();
    _R_enu2ned << 1,0,0,0,-1,0,0,0,-1;
    _Q.setIdentity();
    _Q_dot.setIdentity();
    _Ib << 0.845,0,0,0,0.845,0,0,0,0.912; // Drone's nominal inertia matrix
    _C.setIdentity();
    _M.setIdentity();

    // flag initialization
    _first_imu = false;
	_first_odom = false;

    // output initialization
    estimate[0] = estimate[1] = estimate[2] = estimate[3] = estimate[4] = estimate[5] = 0;
    u[0] = u[1] = 0;
    u[2] = 234.7235; // Thrust of the initial motor speed 2620
    tau[0] = tau[1] = tau[2] = 0;
    
    // Controller and Estimator initialization
    rtObj.initialize();
    est.initialize();

    for (int i = 0; i < 3; i++) {
        rtObj.rtU.position[i] = _p_b(i);
        rtObj.rtU.linear_vel[i] = _p_b_dot(i);
        rtObj.rtU.eta[i] = est.rtU.eta[i] = _eta_b(i);
        rtObj.rtU.eta_dot[i] = est.rtU.eta_dot[i] = _eta_b_dot(i);
        rtObj.rtU.position_des[i] = _pos_ref(i);
        rtObj.rtU.vel_linear_des[i] = est.rtU.p_dot[i] = _pos_ref_dot(i);
        rtObj.rtU.acc_linear_des[i] = _pos_ref_dot_dot(i);
        est.rtU.u[i] = est.rtU.tau[i] = 0;
    }

    rtObj.rtU.psi_d = psi_d;
    rtObj.rtU.dot_psi_d = dot_psi_d;
    rtObj.rtU.ddot_psi_d = ddot_psi_d;

}

Eigen::Matrix3d skew(Eigen::Vector3d v){
	Eigen::Matrix3d skew;
	skew <<    0 , -v(2) , v(1),
		 v(2) , 0   , -v(0),
		-v(1) , v(0) ,    0;
	return skew;
}

void HC_NODE::odom_callback(nav_msgs::Odometry odom) {
	//reads position and linear velocity
	Eigen::Vector3d pos_enu(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);		//world-ENU frame
	Eigen::Vector3d vel_b_enu(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);   // The sensor gives the velocities in body-ENU frame
	
	_p_b = _R_enu2ned*pos_enu;							//transform in world-NED frame
	_p_b_dot = _Rb*_R_enu2ned*vel_b_enu;  				//transform in world-NED frame (first in body-NED then in world-NED)	
	
	_first_odom = true;
}

void HC_NODE::imu_callback (sensor_msgs::Imu imu){
	//reads angular velocity and orientation
	Eigen::Vector3d omega_b_b_enu(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z);    //omega_b_b in body-ENU frame
	_omega_b_b = _R_enu2ned*omega_b_b_enu;								     	//transform in body-NED frame

	double phi, theta, psi;
	
	Eigen::Quaterniond quat(imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z);		//obtain orientation in ENU frame
	Eigen::Matrix3d R = quat.toRotationMatrix();

	_Rb = _R_enu2ned*R*_R_enu2ned.transpose();						//transform in NED frame

	psi = atan2( _Rb(1,0) , _Rb(0,0) );													//extract RPY angles
	theta = atan2( -_Rb(2,0) , sqrt(_Rb(2,1)*_Rb(2,1) + _Rb(2,2)*_Rb(2,2)) );
	phi = atan2( _Rb(2,1),_Rb(2,2) );

	_eta_b(0) = phi;
	_eta_b(1) = theta;
	_eta_b(2) = psi;

    double phi_dot = _eta_b_dot(0);
	double theta_dot = _eta_b_dot(1);

    _Q << 1 ,        0 	,  		   -sin(theta) ,
		  0 ,  cos(phi) ,  cos(theta)*sin(phi) ,
		  0 , -sin(phi) ,   cos(theta)*cos(phi);	
		  

	_Q_dot << 0 ,        0           ,									         -theta_dot*cos(theta),
		      0 ,  -phi_dot*sin(phi) ,    -theta_dot*sin(theta)*sin(phi) + phi_dot*cos(theta)*cos(phi),
		      0 ,  -phi_dot*cos(phi) ,    -theta_dot*sin(theta)*cos(phi) - phi_dot*cos(theta)*sin(phi);	


	_eta_b_dot = _Q.inverse()*_omega_b_b;

	_C = _Q.transpose()*skew(_Q*_eta_b_dot)*_Ib*_Q 	+ _Q.transpose()*_Ib*_Q_dot ;
	_M = _Q.transpose()*_Ib*_Q;

	_first_imu = true;
}

void HC_NODE::ctrl_loop() {
    ros::Rate r(RATE);

    // read reference values from file
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

    // Index for the trajectory vectors
    int ii = 0;

    // Waiting for the first imu and odom
    while (_first_imu == false || _first_odom == false) {
        cout<<"Waiting for sensors\n";
        r.sleep();
    }

    mav_msgs::Actuators cmd;
    cmd.angular_velocities.clear();

    for (int i = 0; i < 4; i++) {
        cmd.angular_velocities.push_back(2620);
    }

    // 5 seconds of open loop for the lift off
    // the estimator runs during this phase
    cout<<"OPEN LOOP START\n";
    double time = 0;
    while (time < 5) {
        _motor_speed_pub.publish(cmd);

        est.rtU.eta[0] = _eta_b(0);
        est.rtU.eta[1] = _eta_b(1);
        est.rtU.eta[2] = _eta_b(2);
        est.rtU.eta_dot[0] = _eta_b_dot(0);
        est.rtU.eta_dot[1] = _eta_b_dot(1);
        est.rtU.eta_dot[2] = _eta_b_dot(2);
        est.rtU.u[0] = u[0];
        est.rtU.u[1] = u[1];
        est.rtU.u[2] = u[2];
        est.rtU.tau[0] = tau[0];
        est.rtU.tau[1] = tau[1];
        est.rtU.tau[2] = tau[2];
        est.rtU.p_dot[0] = _p_b_dot(0);
        est.rtU.p_dot[1] = _p_b_dot(1);
        est.rtU.p_dot[2] = _p_b_dot(2);
        
        est.step();

        cout<<"estimate: \n";
        for (int i = 0; i < 6; i++) {
            estimate[i] = est.rtY.estimate[i];
            cout<<est.rtY.estimate[i]<<endl;
        }
        cout<<endl;

        time = time + 1.0/RATE;
        r.sleep();
    }

    cout<<"OPEN LOOP END\n";

    while(ros::ok()) {

        // current reference values saved in the correct arrays
        _pos_ref(0) = P_x[ii];
        _pos_ref(1) = P_y[ii];
        _pos_ref(2) = P_z[ii];
        psi_d = P_psi[ii];

        _pos_ref_dot(0) = P_d_x[ii];
        _pos_ref_dot(1) = P_d_y[ii];
        _pos_ref_dot(2) = P_d_z[ii];
        dot_psi_d = P_d_psi[ii];

        _pos_ref_dot_dot(0) = P_dd_x[ii];
        _pos_ref_dot_dot(1) = P_dd_y[ii];
        _pos_ref_dot_dot(2) = P_dd_z[ii];
        ddot_psi_d = P_dd_psi[ii];

        // Estimator input
        for (int i = 0; i < 3; i++) {
            est.rtU.eta[i] = _eta_b(i);
            est.rtU.eta_dot[i] = _eta_b_dot(i);
            est.rtU.u[i] = u[i];
            est.rtU.tau[i] = tau[i];
            est.rtU.p_dot[i] = _p_b_dot(i);
        }
        
        // Estimator step
        est.step();

        // Estimator ouput
        for (int i = 0; i < 6; i++) {
            estimate[i] = est.rtY.estimate[i];
        }

        // Controller input
        for (int i = 0; i < 3; i++) {
            rtObj.rtU.position[i] = _p_b(i);
            rtObj.rtU.linear_vel[i] = _p_b_dot(i);
            rtObj.rtU.eta[i] = _eta_b(i);
            rtObj.rtU.eta_dot[i] = _eta_b_dot(i);
            rtObj.rtU.position_des[i] = _pos_ref(i);
            rtObj.rtU.vel_linear_des[i] = _pos_ref_dot(i);
            rtObj.rtU.acc_linear_des[i] = _pos_ref_dot_dot(i);
        }

        rtObj.rtU.psi_d = psi_d;
        rtObj.rtU.dot_psi_d = dot_psi_d;
        rtObj.rtU.ddot_psi_d = ddot_psi_d;

        for (int i = 0; i < 6; i++) {
            rtObj.rtU.estimate[i] = estimate[i];
        }

        // Controller step
        rtObj.step();

        // Controller output
		mav_msgs::Actuators cmd;
        cmd.angular_velocities.clear();

        for (int i = 0; i < 4; i++) {
		    cmd.angular_velocities.push_back(rtObj.rtY.velocities[i]);
            // cout<<"rotor speed "<<i<<": "<<rtObj.rtY.velocities[i]<<endl;
        }

        //Publish all the commands in topics
	    _motor_speed_pub.publish(cmd);


        // DEBUG 
        // cout<<"u: \n";
        // for (int i = 0; i<6; i++) {
        //     cout<<rtObj.rtY.u[i]<<endl;
        // }

        // cout<<"eta_d_dot: \n";
        // for (int i = 0; i<3; i++) {
        //     cout<<rtObj.rtY.eta_d_dot[i]<<endl;
        // }

        // cout<<"eta_d_ddot: \n";
        // for (int i = 0; i<3; i++) {
        //     cout<<rtObj.rtY.eta_d_ddot[i]<<endl;
        // }

        // cout<<"mu_d: \n";
        // for (int i = 0; i<3; i++) {
        //     cout<<rtObj.rtY.mu_d[i]<<endl;
        // }

        cout<<"estimate: \n";
        for (int i = 0; i<6; i++) {
            cout<<est.rtY.estimate[i]<<endl;
        }

        // cout<<"Q: \n";
        // for (int i = 0; i<3; i++) {
        //     cout<<rtObj.rtY.Q[3*i]<<" "<<rtObj.rtY.Q[3*i+1]<<" "<<rtObj.rtY.Q[3*i+2]<<endl;
        // }

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


