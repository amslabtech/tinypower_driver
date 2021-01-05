#include "tinypower_driver/tinypower_driver.h"

namespace tinypower_driver
{

TinypowerDriver::TinypowerDriver(void)
: local_nh_("~")
, fd_(-1)
, is_first_timer_callback_(true)
{
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &TinypowerDriver::cmd_vel_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    local_nh_.param<std::string>("port_name", port_name_, "/dev/tiny");
    local_nh_.param<int>("baud_rate", baud_rate_, 57600);
    local_nh_.param<int>("buffer_size", buffer_size_, 4096);
    local_nh_.param<double>("reconnection_interval", reconnection_interval_, 1.0);
    local_nh_.param<double>("hz", hz_, 40.0);
    local_nh_.param<int>("poll_timeout", poll_timeout_, 5);
    local_nh_.param<std::string>("robot_frame_id", robot_frame_id_, "base_link");
    local_nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    local_nh_.param<bool>("enable_tf", enable_tf_, false);

    timer_ = nh_.createTimer(ros::Duration(1.0 / hz_), &TinypowerDriver::timer_callback, this);
    odom_.pose.pose.orientation = get_quaternion_msg_from_yaw(0);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    open_port(port_name_);
}

TinypowerDriver::~TinypowerDriver(void)
{
    close_port();
}

void TinypowerDriver::process(void)
{
    ros::spin();
}

void TinypowerDriver::cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg)
{
    send_velocity(msg->linear.x, msg->angular.z);
}

void TinypowerDriver::timer_callback(const ros::TimerEvent& e)
{
    if(request_odom()){
        ros::Time now = ros::Time::now();
        if(!is_first_timer_callback_){
            update_odometry((now - last_time_).toNSec() * 1e-9);
            odom_.header.stamp = now;
            odom_pub_.publish(odom_);
            if(enable_tf_){
                publish_odom_tf();
            }
        }else{
            is_first_timer_callback_ = false;
        }
        last_time_ = now;
    }
}

void TinypowerDriver::open_port(const std::string& port_name)
{
    ros::Rate rate(1.0 / reconnection_interval_);
    while(fd_ < 0 && ros::ok()){
        fd_ = open(port_name.c_str(), O_RDWR);
        if(fd_ >= 0){
            ROS_INFO_STREAM("Successfully connected to the device: " + port_name);
        }else{
            ROS_ERROR_STREAM("Failed to open port: " + port_name);
            ROS_ERROR_STREAM("Error " << errno << " from open: " << strerror(errno));
        }
        rate.sleep();
    }

}

void TinypowerDriver::close_port(void)
{
  if(fd_ >= 0){
    close(fd_);
  }
}

void TinypowerDriver::initialize_port(void)
{
    struct termios newtio;
    if(tcgetattr(fd_, &newtio) != 0){
        ROS_ERROR_STREAM("Failed to initialize port: " << port_name_);
        ROS_ERROR_STREAM("Error " << errno << " from tcgetattr: " << strerror(errno));
        exit(-1);
    }
    memset(&newtio, 0, sizeof(newtio));// clear all settings
    newtio.c_cc[VTIME] = 0;// [ds]
    newtio.c_cc[VMIN] = 0;
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;// ignore parity error
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    cfsetspeed(&newtio, baud_rate_);
    tcflush(fd_, TCIFLUSH);
    const int result = tcsetattr(fd_, TCSANOW, &newtio);
    if(result < 0){
        ROS_FATAL("Failed to set serial port attributes");
        close_port();
        exit(-1);
    }

}

void TinypowerDriver::send_velocity(float v, float omega)
{
    std::stringstream ss;
    ss << "VCX" << v << "\n\r" << "VCR" << omega << "\n\r";
    const std::string data = ss.str();
    ROS_INFO_STREAM(data);
    if(write_data(data) <= 0){
        ROS_ERROR("Failed to send data");
    }
}

bool TinypowerDriver::request_odom(void)
{
    const std::string command = "MVV";
    const std::string begin = std::to_string(command[1]);
    const std::string end = "\n";
    std::string data;

    // received data expected to be "$MVV: <vel>, <yawrate>, <icount1>, <icount2>\n"
    const bool result = request_data(command + "\n\r", begin, end, data);
    if(result){
        std::vector<std::string> splitted_data = split(data, "\n\r>$, :");
        for(auto it=splitted_data.begin();it!=splitted_data.end();++it){
            if(*it == command){
                ++it;
                velocity_ = std::stod(*it);
                ++it;
                yawrate_ = std::stod(*it);
                return true;
            }
        }
    }
    return false;
}

void TinypowerDriver::receive_data(void)
{
    while(ros::ok()){
        request_odom();
    }
}

int TinypowerDriver::write_data(const std::string& data)
{
    std::lock_guard<std::mutex> lock(mtx_);
    return ::write(fd_, data.c_str(), strlen(data.c_str()));
}

bool TinypowerDriver::request_data(
    const std::string & command, const std::string& begin, const std::string& end,
    std::string & buffer)
{
    bool is_begin_received = false;

    struct pollfd fds[1];
    fds[0].fd = fd_;
    fds[0].events = POLLIN;

    buffer.clear();

    if(write_data(command) > 0){
        while(buffer.size() < static_cast<unsigned int>(buffer_size_)){
            if(poll(fds, 1, poll_timeout_) == 0){
                ROS_ERROR("Poll reached timeout");
                return false;
            }
            if(fds[0].revents & POLLERR){
                ROS_ERROR("Error on socket");
                return false;
            }
            // use magic number here to avoid warning
            char buffer_c[256];
            int size = 0;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                size = ::read(fd_, buffer_c, 256);
            }
            if(size >= 0){
                buffer.append(buffer_c, size);
            }else{
                ROS_ERROR("Failed to read");
                return false;
            }

            if(!is_begin_received){
                const int pos = buffer.find_first_of(begin);
                if(pos > 0){
                    buffer.erase(0, pos);
                    is_begin_received = true;
                }
            }else{
                const int pos = buffer.find_first_of(end);
                if(pos > 0){
                    buffer.erase(pos + 1, buffer.size() - (pos + 1));
                    return true;
                }
            }
        }
    }
    ROS_ERROR("Failed to receive expected data");
    return false;
}

std::vector<std::string> TinypowerDriver::split(const std::string& str, const std::string& delimiter)
{
    std::vector<std::string> str_vector;
    std::string data = str;
    std::string::size_type pos = str.npos;
    while((pos = data.find_first_of(delimiter)) != str.npos){
        if(pos > 0){
            str_vector.push_back(data.substr(0, pos));
        }
        data = data.substr(pos + 1);
    }
    return str_vector;
}

void TinypowerDriver::update_odometry(double dt)
{
    odom_.header.frame_id = odom_frame_id_;
    odom_.child_frame_id = robot_frame_id_;
    const double yaw = tf2::getYaw(odom_.pose.pose.orientation);
    const double dx = velocity_ * dt;
    const double dyaw = yawrate_ * dt;
    odom_.pose.pose.position.x += dx * cos(yaw);
    odom_.pose.pose.position.y += dx * sin(yaw);
    odom_.pose.pose.orientation = get_quaternion_msg_from_yaw(yaw + dyaw);
    odom_.twist.twist.linear.x = velocity_;
    odom_.twist.twist.angular.z = yawrate_;
}

geometry_msgs::Quaternion TinypowerDriver::get_quaternion_msg_from_yaw(const double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

void TinypowerDriver::publish_odom_tf(void)
{
    geometry_msgs::TransformStamped transform;
    transform.header = odom_.header;
    transform.child_frame_id = odom_.child_frame_id;
    transform.transform.translation.x = odom_.pose.pose.position.x;
    transform.transform.translation.y = odom_.pose.pose.position.y;
    transform.transform.translation.z = odom_.pose.pose.position.z;
    transform.transform.rotation = odom_.pose.pose.orientation;
    tfb_->sendTransform(transform);
}

}
