#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <knm_tiny_power/knm_tiny_lib.h>
#include <knm_tiny_msgs/Velocity.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/thread.hpp>
#include <sys/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

using namespace std;

//calibration data
//float clb_yr = 1.0 - 1.0/24.0;

const string header_frame("odom");
const string child_frame("base_link");

/////////////////////////////////////////////////////////
//-------------------CallBack!!!!!---------------------//
/////////////////////////////////////////////////////////
boost::mutex cmd_mutex_;
knm_tiny_msgs::Velocity cmd_;
void cmdCallback(const knm_tiny_msgs::VelocityConstPtr& msg)
{
	boost::mutex::scoped_lock(cmd_mutex_);
	cmd_=*msg;
}

void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
	boost::mutex::scoped_lock(cmd_mutex_);
    cmd_.op_linear = msg->linear.x;
    cmd_.op_angular = -msg->angular.z;
    std::cout << cmd_ << std::endl;
}

/////////////////////////////////////////////////////////
//-----------------Odometry Calculation----------------//
/////////////////////////////////////////////////////////
void calcOdometry(geometry_msgs::Pose2D& x,
					const nav_msgs::Odometry tiny_state,
					const float dt)
{
	geometry_msgs::Pose2D pre_x = x;
	float v = tiny_state.twist.twist.linear.x;
	//float ang = clb_yr * tiny_state.twist.twist.angular.z + v*0.02;
	float ang = tiny_state.twist.twist.angular.z;
	float dl = v * dt;
	float dtheta = ang * dt;
	float dtheta_2 = dtheta * 0.5;

	if(fabs(dtheta) > 0.0001){
		float row = dl/dtheta;
		//dl = 2 * row * sin(dtheta/2);
		dl = 2 * row * sin(dtheta_2);
	}

	x.theta = pre_x.theta + dtheta;
	//x.theta = piToPI(x.theta);
	//x.x = pre_x.x + dl * cos(pre_x.theta + dtheta/2);
	//x.y = pre_x.y + dl * sin(pre_x.theta + dtheta/2);
	x.x = pre_x.x + dl * cos(pre_x.theta + dtheta_2);
	x.y = pre_x.y + dl * sin(pre_x.theta + dtheta_2);
	cout<<", x="<<x.x<<", y="<<x.y<<endl;
}


/////////////////////////////////////////////////////////
//-------------------Setting!!!!!!---------------------//
/////////////////////////////////////////////////////////
void setState(float MVV[],
				ros::Time current_time,
				nav_msgs::Odometry& tiny_state)
{
	tiny_state.header.frame_id = "odom";
	tiny_state.child_frame_id = "odom";
	tiny_state.header.stamp = current_time;
	tiny_state.pose.pose.position.x = MVV[2];
	tiny_state.pose.pose.position.y = MVV[3];
	tiny_state.pose.pose.position.z = 0;
	tiny_state.twist.twist.linear.x = MVV[0];
	tiny_state.twist.twist.linear.y = 0;
	//tiny_state.twist.twist.angular.z = -MVV[1];
	tiny_state.twist.twist.angular.z = MVV[1];
}

void setOdom(float MVV[],
				geometry_msgs::Pose2D pose,
				ros::Time current_time,
				nav_msgs::Odometry& tiny_odom)
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);
	tiny_odom.header.stamp = current_time;
	tiny_odom.header.frame_id = header_frame;
	tiny_odom.child_frame_id = child_frame;
	tiny_odom.pose.pose.position.x = pose.x;
	tiny_odom.pose.pose.position.y = pose.y;
	tiny_odom.pose.pose.position.z = 0;
	tiny_odom.twist.twist.linear.x = MVV[0];
	tiny_odom.twist.twist.linear.y = 0;
	//tiny_odom.twist.twist.angular.z = -MVV[1];
	tiny_odom.twist.twist.angular.z = MVV[1];
	tiny_odom.pose.pose.orientation = odom_quat;
}

void setTf(geometry_msgs::Pose2D pose,ros::Time current_time,
			geometry_msgs::TransformStamped& odom_trans)
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = header_frame;
	odom_trans.child_frame_id = child_frame;
	odom_trans.transform.translation.x = pose.x;
	odom_trans.transform.translation.y = pose.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
}

/////////////////////////////////////////////////////////
//-------------------Main Loop!!!!!!-------------------//
/////////////////////////////////////////////////////////
void mainLoop()
{
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<nav_msgs::Odometry>("tinypower/state", 10);
	ros::Publisher pub2 = n.advertise<nav_msgs::Odometry>("/odom", 10);
	ros::Publisher pub_cur = n.advertise<std_msgs::Float32>("tinypower/cur", 10);
	ros::Publisher pub_vol = n.advertise<std_msgs::Float32>("tinypower/vol", 10);
	ros::Subscriber sub1 = n.subscribe("tinypower/command_velocity", 10, cmdCallback);
	ros::Subscriber sub2 = n.subscribe("/cmd_vel", 1, cmdVelCallback);

	tf::TransformBroadcaster odom_broadcaster;

	//Constractor
	// TinyInterface device("/dev/tiny", 9600);
	//TinyInterface device("/dev/ttyUSB1", 57600);
	//TinyInterface device("/dev/ttyS0", 57600);
	//TinyInterface device("/dev/ttyS0", 19200);
	// TinyInterface device("/dev/tinyUSB", 19200);
	TinyInterface device("/dev/tiny", 57600);
	geometry_msgs::Pose2D pose;
	pose.x=0;	pose.y=0;	pose.theta=0;
	ros::Time current_time;
	current_time = ros::Time::now();
	std_msgs::Float32 cur_sum;
	std_msgs::Float32 vol_msg;

	struct timeval s, e;
	float dt = 0;

	float cur[2];
	float vol[1];

	ros::Rate r(40);
	while(ros::ok()){
        std::cout<< "knm tiny power!!!" << std::endl;
		gettimeofday(&s, NULL);
		current_time = ros::Time::now();
		float MVV[4];
		knm_tiny_msgs::Velocity cmd;
		{
			boost::mutex::scoped_lock(odo_mutex_);
			cmd = cmd_;
		}

		//-----------Interface fanction with Tiny Power!!!!!------------

		//Receive DATA from Tiny
		device.recvData(MVV);
		device.MCUR(cur);
		device.MPV(vol);
		cur_sum.data = cur[0] + cur[1];
		vol_msg.data = vol[0];
		//Send Motion Command
		device.motionControl(cmd.op_linear, cmd.op_angular);
		//--------------------------------------------------------------

		cout<<"V="<<cmd.op_linear<<", ANG="<<cmd.op_angular;
		cout<<", actV="<<MVV[0]<<", actANG="<<MVV[1];
		cout<<", Ir="<<cur[0]<<", Il="<<cur[1];
		cout<<", V="<<vol[0];

		//State pulish
		nav_msgs::Odometry tiny_state;
		setState(MVV, current_time, tiny_state);

		//Odometry calculation
		calcOdometry(pose, tiny_state, dt);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);

		//Odometry publish
		nav_msgs::Odometry tiny_odom;
		setOdom(MVV, pose, current_time, tiny_odom);

		//TF publish
		geometry_msgs::TransformStamped odom_trans;
		setTf(pose, current_time, odom_trans);


		//--------------publish----------------//
		odom_broadcaster.sendTransform(odom_trans);
		pub1.publish(tiny_state);
		pub2.publish(tiny_odom);
		pub_cur.publish(cur_sum);
		pub_vol.publish(vol_msg);

		ros::spinOnce();
		r.sleep();

		gettimeofday(&e, NULL);
		dt = (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6;
		//printf("time = %lf\t", dt);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "knm_tiny_power");
	mainLoop();
	return 0;
}
