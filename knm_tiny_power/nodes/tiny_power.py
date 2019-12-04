#!/usr/bin/env python
# cofing: UTF-8

import rospy

import serial
import sys
import threading
import time
import numpy as np


class TinyPower:
	def __init__(self, dev_name, baudrate):
		self.dev_name = dev_name
		self.baud = baudrate
		self.ser = serial.Serial(self.dev_name, self.baud, timeout=0.1)
		self.linear = 0.0
		self.angular = 0.0

		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.currTime = time.time()
		self.prevTime = self.currTime

		self.global_lock = threading.Lock()

		self.thread1 = threading.Thread(target=self.get_char_loop)
		self.thread1.start()

	def commVCX(self, linear):
		linear = str(round(linear, 2))
		#print 'linear : ', linear
		command = 'VCX'+linear+'\n\r'
		#self.global_lock.acquire()
		self.ser.write(command)
		#self.global_lock.release()

	def commVCR(self, angular):
		angular = str(round(angular, 2))
		#print 'angular : ', angular
		command = 'VCR'+angular+'\n\r'
		#self.global_lock.acquire()
		self.ser.write(command)
		#self.global_lock.release()

	def commMVV(self):
		#self.global_lock.acquire()
		self.ser.write('MVV\n\r')
		#self.global_lock.release()
		#line = self.ser.readline()
		#ch = self.ser.read()
		#print '%c'%(self.ser.read()),
		#sys.stdout.write(ch)
		#print '[%c] , [%d]'%(ch, ord(ch))
		#while ch != "$":
		#	ch = self.ser.read()
		#	print '[%c] , [%d]'%(ch, ord(ch))
		#line = self.ser.readline()
		#print line
		#print line
		#print line[0], line[1]
		#print self.hantei(line)
		#while not self.hantei(line):
		#	line = self.ser.readline()
		#	print line
		#print line
		#while True:
		#	if line[0:5] == '$MVV:':
		#		print 'hey'
		#		break
		#	else:
		#		print 'oh'
		#		line = self.ser.readline()
		#		print line

	def calcOdom(self):
		pre_x = self.x
		pre_y = self.y
		pre_theta = self.theta
		dt = self.currTime-self.prevTime
		dl = self.linear*dt
		dtheta = self.angular*dt
		dtheta_2 = dtheta*0.5

		if abs(dtheta)>0.0001:
			row = dl/dtheta
			dl = 2*row*np.sin(dtheta_2)

		self.theta = pre_theta + dtheta
		self.x = pre_x + dl*np.cos(pre_theta + dtheta_2)
		self.y = pre_y + dl*np.sin(pre_theta + dtheta_2)


	def get_char_loop(self):
		print "Thread Started"
		while not rospy.is_shutdown():
			#self.global_lock.acquire()
			ch = self.ser.read()
			#self.global_lock.release()
			#sys.stdout.write(ch)
			if ch == "$":
				#self.global_lock.acquire()
				line = self.ser.readline()
				#print line
				#self.global_lock.release()
				if line[0:3] == 'MVV':
					self.currTime = time.time()
					data = line.split(':')
					data = data[1].split('\r')
					data = data[0].split(',')
					self.linear = float(data[0])
					self.angular = float(data[1])
					self.calcOdom()
					#print "%f, %f"%(self.x, self.y)
					#print data
					self.prevTime = self.currTime
				else:
					pass
					#print line
		print "Thread Finished"







