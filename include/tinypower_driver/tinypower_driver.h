#ifndef __TINYPOWER_DRIVER_TINYPOWER_DRIVER_H
#define __TINYPOWER_DRIVER_TINYPOWER_DRIVER_H

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

namespace tinypower_driver
{

class TinypowerDriver
{
public:
    TinypowerDriver(void);
    ~TinypowerDriver(void);
    void process(void);
    void cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg);
    void timer_callback(const ros::TimerEvent& e);
    void open_port(const std::string& port_name);
    void close_port(void);
    void initialize_port(void);
    void send_velocity(float v, float omega);
    bool request_odom(void);
    void receive_data(void);
    int write_data(const std::string& data);
    bool request_data(
        const std::string & command, const std::string& begin, const std::string& end,
        std::string & buffer);
    std::vector<std::string> split(const std::string& str, const std::string& delimiter);
    void update_odometry(double dt);
    geometry_msgs::Quaternion get_quaternion_msg_from_yaw(const double yaw);
    void publish_odom_tf(void);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer timer_;
    // port name e.g. /dev/ttyUSB0
    std::string port_name_;
    int baud_rate_;
    int buffer_size_;
    double hz_;
    // interval time to try reconnection [s]
    double reconnection_interval_;
    // timeout for poll [ms]
    int poll_timeout_;
    std::string robot_frame_id_;
    std::string odom_frame_id_;
    bool enable_tf_;
    // file descriptor
    int fd_;
    std::shared_ptr<std::thread> receiving_thread_;
    std::mutex mtx_;
    double velocity_;
    double yawrate_;
    nav_msgs::Odometry odom_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    bool is_first_timer_callback_;
    ros::Time last_time_;
};

}

#endif // __TINYPOWER_DRIVER_TINYPOWER_DRIVER_H
