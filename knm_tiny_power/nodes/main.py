#!/usr/bin/env python
import roslib; roslib.load_manifest('knm_tiny_power')
import rospy

from nav_msgs.msg import Odometry
from knm_tiny_msgs.msg import Velocity
from geometry_msgs.msg import Twist

import time
from time import sleep
import serial
import threading
import math

import tiny_power
import tf

# tinypower = tiny_power.TinyPower('/dev/tiny', 115200)
tinypower = tiny_power.TinyPower('/dev/tiny', 57600)
# tinypower = tiny_power.TinyPower('/dev/tiny', 38400)
# tinypower = tiny_power.TinyPower('/dev/tiny', 19200)
# tinypower = tiny_power.TinyPower('/dev/tiny', 9600)

global_lock = threading.Lock()
####
####
def commandCallback(data):
	#global_lock.acquire()
        print data
	tinypower.commVCX(data.op_linear)
	tinypower.commVCR(data.op_angular)
	#global_lock.release()
	pass
####	global global_lock
####	print 'linear : ', data.op_linear
####	#global_lock.acquire()
####	#ser.write('VCX'+str(data.op_linear)+'\n\r'+'VCR'+str(data.op_angular)+'\n\r')
####	ser.write('VCX'+str(round(data.op_linear, 2))+'\n\r')
####	#line = ser.readline()
####	#line = ser.readline()
####	print 'VCX'+str(round(data.op_linear, 2))+'\n\r'
####	#print line
####	ser.write('VCR'+str(round(data.op_angular, 2))+'\n\r')
####	#global_lock.release()
####	line = ser.readline()
####	line = ser.readline()
####	print 'angular : ', data.op_angular
####	#print 'VCR'+str(round(data.op_angular, 2))+'\n\r'
####	#print line
####
####
####def calcOdometry(pose, data, dt):
####	pass
def commandVelCallback(data):
	#global_lock.acquire()
        print data
	tinypower.commVCX(data.linear.x)
	tinypower.commVCR(data.angular.z)
	#global_lock.release()
	pass

if __name__ == '__main__':
	rospy.init_node('knm_tiny_power')


	command_sub = rospy.Subscriber('tinypower/command_velocity', Velocity, commandCallback)
        command_vel_sub = rospy.Subscriber('/cmd_vel', Twist, commandVelCallback)
	odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 10)

	#ser = serial.Serial('/dev/ttyUSB2', 57600)
	#ser = serial.Serial('/dev/tiny', 57600)
	#ser = serial.Serial('/dev/tiny', 115200)

	odom_data = Odometry()

	r = rospy.Rate(40)

	####global global_lock

        print "MAIN.PY START"

	while not rospy.is_shutdown():
		tinypower.commMVV()

		odom_data.header.stamp = rospy.Time.now()
                odom_data.header.frame_id = "odom"
                odom_data.child_frame_id = "base_link"
		odom_data.pose.pose.position.x = tinypower.x
		odom_data.pose.pose.position.y = tinypower.y
		# odom_data.pose.pose.orientation.z = tinypower.theta
                (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0, 0, tinypower.theta)
                odom_data.pose.pose.orientation.x = qx
                odom_data.pose.pose.orientation.y = qy
                odom_data.pose.pose.orientation.z = qz
                odom_data.pose.pose.orientation.w = qw
		odom_data.twist.twist.linear.x = tinypower.linear
		odom_data.twist.twist.angular.z = tinypower.angular
		odom_pub.publish(odom_data)
		#print "%5.2f, %5.2f, %5.2f"%(tinypower.x, tinypower.y, tinypower.theta)

		r.sleep()
	print "Programm is finished (main.py)"
	#tinypower.stop()
	rospy.spin()


