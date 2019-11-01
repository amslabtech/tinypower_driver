#!/usr/bin/env python
import roslib; roslib.load_manifest('ceres_amu')
import rospy
from time import sleep
import select
import subprocess
import threading
import os
from threading import Thread
from serial import Serial
from ceres_msgs.msg import * 
from std_msgs.msg import Bool 

count = 0

class MySerial(Serial):
	
	#Wrapper for Serial
	try:
		import io
	except ImportError:
		# serial.Serial inherits serial.FileLike
		pass
	else:
		def readline(self):
		
		#Overrides io.RawIOBase.readline which cannot handle with '\r' delimiters
		
			ret = ''
			while True:
				c = self.read(1)
				if c == '':
					return ret
				elif c == '\r':
					return ret + c
				else:
					ret += c
				

def amuCommunicate(pub):
	r = rospy.Rate(50) 
	global count
	count = 0
	error_count = 0
	try:
		#ser = MySerial('/dev/ttyUSB0', 38400, timeout=1)
		ser = MySerial('/dev/USBamu', 38400, timeout=1)
		#ser = MySerial('/dev/USBmicrostrain', 38400, timeout=1)
		#ser = MySerial('/dev/tiny', 38400, timeout=1)
		print"AMU connected!!!"
	except:
		print"Can not connect AMU!!!"
		exit(1)

	#Mode Change(float output)
	ser.write('t05020707\r')
	while True:
		count+=1
		#try:
		#	str=ser.readline()
		#except :
		#	continue
		str=ser.readline()
		data=str.split(',')
		amu=AMU_data()
		if len(data)==10:
			try:
	    			amu.roll= float(data[0])
			except ValueError:
				print "-------------error roll-------------"
				r.sleep()
				continue
			try:
	    			amu.pitch= -float(data[1])
			except ValueError:
				print "-------------error pitch-------------"
				r.sleep()
				continue
			try:
	    			amu.yaw= -float(data[2])
			except ValueError:
				print "-------------error yaw-------------"
				r.sleep()
				continue
			try:
	    			amu.droll= float(data[3])
			except ValueError:
				print "-------------error droll-------------"
				r.sleep()
				continue
			try:
	    			amu.dpitch= -float(data[4])
			except ValueError:
				print "-------------error dpitch-------------"
				r.sleep()
				continue
			try:
	    			amu.dyaw= -float(data[5])
			except ValueError:
				print "-------------error dyaw-------------"
				r.sleep()
				continue
			try:
				amu.xaccel=float(data[6])
			except ValueError:
				print "-------------error xaccel-------------"
				r.sleep()
				continue
			try:
				amu.yaccel=float(data[7])
			except ValueError:
				print "-------------error yaccel-------------"
				r.sleep()
				continue
			try:
				amu.zaccel=float(data[8])
			except ValueError:
				print "-------------error zaccel-------------"
				r.sleep()
				continue

			error_count = 0
			if ((count%5)==0):
				print 'Roll[deg],Pitch[deg],Yaw[deg],droll[deg/s],dpitch[deg/s],dyaw[deg/s],xaccel[G],yaccel[G],zaccel[G],status'
				print "%8.2f" % amu.roll, "%8.2f" % amu.pitch, "%8.2f" % amu.yaw, "%8.2f" % amu.droll, "    %8.2f" % amu.dpitch, "     %8.2f" % amu.dyaw, "   %8.2f" % amu.xaccel, "%8.2f" % amu.yaccel, "   %8.2f" % amu.zaccel, "   %s" % data[9]
				#rospy.loginfo( data )
			
			amu.header.stamp=rospy.Time.now()
			#print amu
			pub.publish(amu)
			
			#Error Massage
			if data[9]!='0x0000\r':
				if data[9]=='0x0000\r':
					print '====Error of the X axis gyro====='
				elif data[9]=='0x0001\r':
					print '====Error of the Y axis gyro====='
				elif data[9]=='0x0002\r':
					print '====Error of the Z axis gyro====='
				elif data[9]=='0x0003\r':
					print '====Error of the source voltage====='
				elif data[9]=='0x0004\r':
					print '====Error of the 5V voltage====='
				elif data[9]=='0x0005\r':
					print '====Error of the 3.3V voltage====='
				elif data[9]=='0x0006\r':
					print '====Error of the X axis acceleration sensor====='
				elif data[9]=='0x0007\r':
					print '====Error of the Y axis acceleration sensor====='
				elif data[9]=='0x0008\r':
					print '====Error of the Z axis acceleration sensor====='
				elif data[9]=='0x0009\r':
					print '====Error of the temperature sensor====='
				else:
					print '====Error of the unknown source====='
		else:
			print 'Data array is broken!!!!!!!!'
			error_count += 1
			if error_count>3:
				break
				
		r.sleep()
		
	print"exitting amu!!"
	
def errorCheck(pub):
	r = rospy.Rate(10)
	global count
	precount = 0
	thread_unloop_count = 0
	while True:
		error_flag = Bool()
		if (count - precount) > 0:
			thread_unloop_count = 0
			precount = count
		elif (count - precount) == 0:
			thread_unloop_count += 1
			precount = count
		else:
			thread_unloop_count = 0
			precount = 0
			
		if thread_unloop_count > 2:
			error_flag.data = True
			print'********++++++++++------------------AMU is rebooting now!!------------------++++++++******'
		else:
			error_flag.data = False
			print'********++++++++++------------------AMU is OK!!------------------++++++++******'
			
		pub.publish(error_flag)
		r.sleep()


if __name__ == '__main__':
	rospy.init_node('ceres_amu')
	pub = rospy.Publisher('/AMU_data', AMU_data)
	pub2 = rospy.Publisher('/AMU_error', Bool)
	r = rospy.Rate(2) 
	
	amu=threading.Thread(target=amuCommunicate, name="amu", args=(pub,))
	amu.setDaemon(True)
	amu.start()
	
	error_check=threading.Thread(target=errorCheck, name="error_check", args=(pub2,))
	error_check.setDaemon(True)
	error_check.start()
	
	main_thread=threading.currentThread()
	
	while not rospy.is_shutdown():
		amu_run_flag=False
		tlist=threading.enumerate()
		for t in tlist:
			if t is main_thread: continue
			if t is amu:	amu_run_flag=True
		
		if amu_run_flag==False:
			print "Re-start!!!"
			amu=threading.Thread(target=amuCommunicate, name="amu", args=(pub,))
			amu.setDaemon(True)
			amu.start()
			sleep(2)
			
		r.sleep()
	rospy.spin()

