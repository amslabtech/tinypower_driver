#include <ros/ros.h>
#include <knm_tiny_msgs/Velocity.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Joy.h>

using namespace std;



/////////////////////////////////////////////////////////
//-------------------CallBack!!!!!---------------------//
/////////////////////////////////////////////////////////
boost::mutex joy_mutex_;
knm_tiny_msgs::Velocity cmd_;
double gain = 0;
void joycallback(const sensor_msgs::JoyConstPtr& msg)
{
	boost::mutex::scoped_lock(joy_mutex_);

	//gain = (msg->axes[2]+1.0) * (1-msg->buttons[8]*0.5) * (1+msg->buttons[10]*0.6);
	gain = (1-msg->buttons[8]*0.5) * (1+msg->buttons[10]*0.6);
	cmd_.op_linear=msg->axes[1]*0.8*gain;
	cmd_.op_angular=-msg->axes[0]*1.2*gain;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "knm_tiny_joy");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/joy", 10, joycallback);
	ros::Publisher pub = n.advertise<knm_tiny_msgs::Velocity>("tinypower/command_velocity", 10);
	
	ros::Rate r(20);
	while(ros::ok()){
		pub.publish(cmd_);
		
		ros::spinOnce();
		r.sleep();
		
	}
}

