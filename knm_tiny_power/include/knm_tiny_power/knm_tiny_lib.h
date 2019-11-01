//
//
//

#ifndef TINY_CLASS_H
#define TINY_CLASS_H

#include <ros/ros.h>
#include <cereal_catkin/CerealCatkin.h>
#include <boost/thread.hpp>
#include <string>
#include <list>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <iomanip>
using namespace std;
class TinyInterface : public cereal::CerealCatkin{
	private:
		list<string> split(string str, string delim);
		void readData(const char* command, string& reply);
		void dataStorage(list<string> strList, const char* command, float data[], const int n);
		void RGParam(list<string> strList_RG, float RG[]);
		void VGParam(list<string> strList_VG, float VG[]);
	
	public:
		TinyInterface(const char * port_name, int baud_rate = 57600);
		void recvParam(float RG1[], float RG2[], float VG[]);
		void recvData(float MVV[]);
		void MCUR(float data[]);
		void MPV(float data[]);
		//void recvData(float ME[], float MVV[]);
		void outputPWM(float du1,float du2);
		void motionControl(float v,float yawrate);
		void revControl(int id,float rad);
		void immediateStop();
	
};

#endif // TINY_CLASS_H


