#!/usr/bin/env python
# coding: UTF-8

import roslib; roslib.load_manifest('knm_tiny_power')
from nav_msgs.msg import *
import rospy
from subprocess import *
import string
import time
from struct import *


act_node_list_=[]
tiny_odom = Odometry()
callback_count = 0
pre_count = 0
thread_unloop_count = 0

def tinyOdomCallback(data):
	global tiny_odom
	global callback_count
	tiny_odom = data
	callback_count += 1

def setActNodeList():
	act_node_file=open('/home/amsl/ros_catkin_ws/src/hardware/knm_tiny_power/nodes/lists/knm_tiny_list.txt')
	act_node_list=act_node_file.readlines()	
	for line in act_node_list:
		node_info=line.split('\n')
		node_info=node_info[0].split(',')
		if len(node_info)==3 and node_info[0]!='#':
			check=node_info[1].split('.')
			if len(check)>1 and check[1]=='launch':
				node_info={'pkg':node_info[0],'launch':node_info[1],'node':'/'+node_info[2]}
			else:				
				node_info={'pkg':node_info[0],'exe':node_info[1],'node':'/'+node_info[2]}
			act_node_list_.append(node_info)
        print "act_node_list:"
	print act_node_list_


def killNode(node_info):
	kill=Popen(['rosnode','kill',node_info['node']])
	kill.wait()

def nodeList():
	#print '######rosnode list'
	nodelisthandler=Popen(['rosnode','list'],stdout=PIPE)	
	nodelisthandler.wait() 

	#print '######rosnode set'
	nodelist=nodelisthandler.communicate() 
	nd=nodelist[0]
	nd=nd.split("\n")
	#print nd
	return nd


def callbackCheck():
	global callback_count
	global pre_count
	global thread_unloop_count
	
	if (callback_count - pre_count) > 0:
		thread_unloop_count = 0
		pre_count = callback_count
		#print'********++++++++++------------------Msg is OK!!------------------++++++++******'
	elif (callback_count - pre_count) == 0:
		thread_unloop_count += 1
		pre_count = callback_count
		print'********++++++++++------------------Msg is lost!!------------------++++++++******'
	else:
		thread_unloop_count = 0
		pre_count = 0
			
	if thread_unloop_count > 6:
		print'********++++++++++------------------Kill!!------------------++++++++******'
		return True
	else:
		return False
	

def nodeCheck(node_info):
	nd = nodeList()
	for node_name in nd:
		if node_info['node']==node_name:
			#print 'Find node:'+node_info['node']
			return False	
	print 'Not find node:'+node_info['node']
	return True


def runNode(node_info):
	if node_info.has_key('exe'):
		run=Popen(['rosrun',node_info['pkg'],node_info['exe']])
		print "run:",node_info['exe']
	elif node_info.has_key('launch'):
		run=Popen(['roslaunch',node_info['pkg'],node_info['launch']])
	time.sleep(1)


def allKill():
	print'All Killing'
	for node_name in act_node_list_:
		killNode(node_name)
	

def nodeManager():

	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		for node_name in act_node_list_:
			if nodeCheck(node_name):
				runNode(node_name)
			elif callbackCheck():
				allKill()
				runNode(node_name)
				time.sleep(1)
		r.sleep()
	rospy.spin()		

	
if __name__ == "__main__":
	
	rospy.init_node('knm_tiny_revival')
	rospy.Subscriber("tinypower/odom", Odometry, tinyOdomCallback)
	setActNodeList()
	nodeManager()
	allKill()
