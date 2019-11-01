#include <ros/ros.h>
#include <math.h>
#include <boost/thread.hpp>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

nav_msgs::Odometry odom_in;

boost::mutex odom_mutex;
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odom_mutex);
	odom_in = *msg;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_odom");
	ros::NodeHandle n;

	ros::Subscriber sub_odom = n.subscribe("tinypower/odom", 1, odomCallback);
	static tf::TransformBroadcaster br_odom;
	tf::Transform transform_odom;
	tf::Quaternion q_odom;
	q_odom.setRPY(0, 0, 0);
	transform_odom.setRotation(q_odom);
	br_odom.sendTransform(tf::StampedTransform(transform_odom, ros::Time::now(), "tiny_odom", "base_link"));

	static tf::TransformBroadcaster br_laser;
	tf::Transform transform_laser;
	transform_laser.setOrigin( tf::Vector3(0.56, 0.0, 0.76) );
	tf::Quaternion q_laser;
	q_laser.setRPY(0.0, 0.0, 0.0);
	transform_laser.setRotation(q_laser);
	br_laser.sendTransform(tf::StampedTransform(transform_laser, ros::Time::now(), "base_link", "3dlaser"));

	ros::Rate loop_rate(10);

	while(ros::ok()){

		transform_odom.setOrigin(tf::Vector3(
			odom_in.pose.pose.position.x,
			odom_in.pose.pose.position.y,
			odom_in.pose.pose.position.z)
		);
			
		transform_odom.setRotation(tf::Quaternion(
			odom_in.pose.pose.orientation.x,
			odom_in.pose.pose.orientation.y,
			odom_in.pose.pose.orientation.z,
			odom_in.pose.pose.orientation.w)
		);
			
		br_odom.sendTransform(tf::StampedTransform(transform_odom, ros::Time::now(), "tiny_odom", "base_link"));
		br_laser.sendTransform(tf::StampedTransform(transform_laser, ros::Time::now(), "base_link", "3dlaser"));

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

