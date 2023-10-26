#include "ros/ros.h"
#include "boost/thread.hpp"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

#define RATE 100

using namespace std;

class CUBETFPUB {
	public:
		CUBETFPUB();
		void cb(gazebo_msgs::ModelStates::ConstPtr msg);
		void pub_tf();
	private:
		double x;
		double y;
		double z;
		double o_w;
		double o_x;
		double o_y;
		double o_z;
		bool _active;

		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Rate _rate;
};

CUBETFPUB::CUBETFPUB(): _rate(RATE) {
	_topic_sub = _nh.subscribe("/gazebo/model_states", 1, &CUBETFPUB::cb, this);
	x = y = z = 0;
	o_w = 1;
	o_x = o_y = o_z = 0;
	_active = false;

	boost::thread(&CUBETFPUB::pub_tf, this);
}

//Callback function: the input of the function is the data to read
void CUBETFPUB::cb(gazebo_msgs::ModelStates::ConstPtr msg) {
	bool cube_exist = false;
	int cube_pos = -1;
	// int N = sizeof(msg->name)/sizeof(msg->name[0]);
	// cout<<"There are "<<N<<" models\n";
	int N = 4;
	for (int i = 0; i < N; i++) {
		char t;
		char temp[100];
		int j = 0;
		do {
			t = msg->name[i][j];
			// cout<<"t: "<<t<<endl;
			temp[j] = t;
			j++;
		} while (t != '\0');

		// cout<<temp<<endl;
		
		if (strcmp("cube", temp) == 0) {
			cube_exist = true;
			cube_pos = i;
		}
	}

	cout<<cube_pos<<endl;

	if (cube_exist) {
		x = msg->pose[cube_pos].position.x;
		y = msg->pose[cube_pos].position.y;
		z = msg->pose[cube_pos].position.z;
		o_w = msg->pose[cube_pos].orientation.w;
		o_x = msg->pose[cube_pos].orientation.x;
		o_y = msg->pose[cube_pos].orientation.y;
		o_z = msg->pose[cube_pos].orientation.z;
		if (!_active) {
			_active = true;
		}
	}
}

void CUBETFPUB::pub_tf() {
	tf::Transform cube_trans;
	tf::TransformBroadcaster trans_br;
	while (ros::ok()) {
		cube_trans.setOrigin(tf::Vector3(x, y, z));
		tf::Quaternion q(o_x, o_y, o_z, o_w);
		cube_trans.setRotation(q);
		trans_br.sendTransform(tf::StampedTransform(cube_trans, ros::Time::now(), "world", "cube_frame"));
		_rate.sleep();
	}
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "CubeTfPub");
	ROS_INFO("CUBE TF PUB STARTED\n");

	//Create the sensor filter class object
	CUBETFPUB ctf;
	ROS_INFO("CONSTRUCTOR WORKED\n");

	ros::spin();

	return 0;
}
