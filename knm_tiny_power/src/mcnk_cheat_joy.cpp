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
bool pub_flag = false;
double gain = 0;
void joycallback(const sensor_msgs::JoyConstPtr& msg)
{
	boost::mutex::scoped_lock(joy_mutex_);

	//gain = (msg->axes[2]+1.0) * (1-msg->buttons[8]*0.5) * (1+msg->buttons[10]*0.6);
	gain = (1-msg->buttons[8]*0.5) * (1+msg->buttons[10]*0.6);

	if( msg->buttons[4] || msg->buttons[5] || msg->buttons[6] || msg->buttons[7] ){
		cmd_.op_linear  = (msg->buttons[4]-msg->buttons[6])*0.5*gain;
		cmd_.op_angular = (msg->buttons[5]-msg->buttons[7])*0.4*gain;
	}else{
		cmd_.op_linear=msg->axes[1]*1.0*gain;
		cmd_.op_angular=-msg->axes[0]*1.2*gain;
	}

	if( msg->buttons[0] ){ pub_flag = true; }
	if( msg->buttons[3] ){ pub_flag = false; }

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mcnk_cheat_joy");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/joy", 10, joycallback);
	ros::Publisher pub = n.advertise<knm_tiny_msgs::Velocity>("tinypower/cheat_velocity", 10);
	
	ros::Rate r(20);
	while(ros::ok()){
		if(pub_flag){
			pub.publish(cmd_);
		}
		
		ros::spinOnce();
		r.sleep();
		
	}
}

