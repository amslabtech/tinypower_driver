#include <ros/ros.h>
#include <stdio.h>
#include <knm_tiny_power/knm_tiny_lib.h>
#define REPLY_SIZE 256
#define TIMEOUT 1000
ostringstream oss;
TinyInterface::TinyInterface(const char * port_name, int baud_rate)
{
	try{ open(port_name, baud_rate); }
	catch(cereal::Exception& e)
	{
		ROS_FATAL("Failed to open the serial port!!!");
		exit(1);
	}
	ROS_INFO("The serial port is opened.");
}


list<string> TinyInterface::split(string str, string delim)
{
    list<string> result;
    int cutAt;
	//printf("-------split start-------\n");
    while( (cutAt = str.find_first_of(delim)) != str.npos ){
        if(cutAt > 0)
        {
            result.push_back(str.substr(0, cutAt));
		//printf("bbbbbbbb\n");
		//printf((str.substr(0, cutAt)).c_str());
		//printf("  ");
        }
        str = str.substr(cutAt + 1);
    }
    if(str.length() > 0){
        result.push_back(str);
	//printf("aaaaaaa\n");
	//printf(str.c_str());
    }
	
	//printf("-------split end-------\n");
	return result;
}

void TinyInterface::readData(const char* command, string& reply)
{
	//write(command);
	std::string reply_str;
	try{
		write(command);
		readBetween(&reply_str, command[1],'\n',TIMEOUT);
		//printf(reply_str.c_str());
		reply = reply_str;
		//read(reply, REPLY_SIZE, TIMEOUT);
	}catch(cereal::TimeoutException& e){
		ROS_ERROR("Read waiting Timeout!");
		motionControl(0,0);
		//outputPWM(0,0);
		//exit(1);
	}
	//printf(reply);
}

void TinyInterface::dataStorage(list<string> strList, 
								const char* command, 
								float data[], 
								const int n)
{
	list<string>::iterator iter = strList.begin(); 
	while( iter != strList.end() ){
		if( !strcmp((*iter).c_str(), command) ) {
			for(int i=0; i<n; i++){
				++iter;
				if(iter == strList.end()){
					ROS_ERROR("end of the strList @dataStorage");
					break;
				}
				data[i] = atof((*iter).c_str());
			}
			break;
		}
		++iter;
	}
}

////////////////////////////////////////////////////////////////////////////
/*-------------------------Public function--------------------------------*/
////////////////////////////////////////////////////////////////////////////
void TinyInterface::outputPWM(float du1,float du2)
{
	oss.clear(); // 状態をクリア.
	oss.str(""); // 文字列をクリア.
	oss<< "P" << 1 << "DU" << du1 <<"\n\r"<< "P" << 2 << "DU" << du2 <<"\n\r";
	string str = oss.str();
	//write(str.c_str());
	try{
		write(str.c_str());
	}catch(cereal::TimeoutException& e){
		ROS_ERROR("write ERROR");
		exit(1);
	}

	//printf(str.c_str());
}

void TinyInterface::motionControl(float v,float yawrate)
{
	oss.clear();
	oss.str("");
	oss<< "VCX" << v <<"\n\r"<< "VCR" << yawrate <<"\n\r";
	string str = oss.str();
	//write(str.c_str());
	try{
		write(str.c_str());
	}catch(cereal::TimeoutException& e){
		ROS_ERROR("write ERROR");
		exit(1);
	}

	//printf(str.c_str());
}

void TinyInterface::revControl(int id,float rad)
{
	oss.clear();
	oss.str("");
	oss<< "RC" << id <<"RS"<< rad <<"\n\r";
	string str = oss.str();
	write(str.c_str());
	//printf(str.c_str());
}

void TinyInterface::immediateStop()
{
	oss.clear();
	oss.str("");
	oss<< "VCIS" << "\n\r";
	string str = oss.str();
	write(str.c_str());
	//printf(str.c_str());
}


void TinyInterface::recvData(float MVV[])
{
	const int size_MVV = 4;
	string reply_MVV;
	readData("MVV\n\r", reply_MVV);
    list<string> strList_MVV = split(reply_MVV, "\n\r>$, :");
	dataStorage(strList_MVV, "VV", MVV, size_MVV);
	
}

void TinyInterface::MCUR(float data[])
{
	const int size_MCUR = 2;
	string reply_MCUR;
	readData("MCUR\n\r", reply_MCUR);
	list<string> strList_MCUR = split(reply_MCUR, "\n\r>$, :");
	dataStorage(strList_MCUR, "CUR", data, size_MCUR);
}

void TinyInterface::MPV(float data[])
{
	const int size_MPV = 1;
	string reply_MPV;
	readData("MPV\n\r", reply_MPV);
	list<string> strList_MPV = split(reply_MPV, "\n\r>$, :");
	dataStorage(strList_MPV, "PV", data, size_MPV);
}

/*
void TinyInterface::recvData(float ME[], float MVV[])
{
	const int size_MVV = 2;
	const int size_ME = 4;
	string reply_MVV, reply_ME;
	readData("MVV\n\r", reply_MVV);
	readData("ME\n\r", reply_ME);
    list<string> strList_MVV = split(reply_MVV, "\n\r>$, :");
    list<string> strList_ME = split(reply_ME, "\n\r>$, :");
	dataStorage(strList_MVV, "VV", MVV, size_MVV);
	dataStorage(strList_ME, "E", ME, size_ME);
	
}*/


void TinyInterface::RGParam(list<string> strList_RG, float RG[])
{
	list<string>::iterator iter = strList_RG.begin(); 
	while( iter != strList_RG.end() ){
		char moji[100];
		sprintf(moji,"%s",(*iter).c_str());
		if( !strncmp("PR", moji, 2) ) {
			RG[0] = atof(&moji[2]);
		}else if( !strncmp("GR", moji, 2) ){
			RG[1] = atof(&moji[2]);
		}else if( !strncmp("KP", moji, 2) ){
			RG[2] = atof(&moji[2]);
		}else if( !strncmp("KI", moji, 2) ){
			RG[3] = atof(&moji[2]);
		}else if( !strncmp("KD", moji, 2) ){
			RG[4] = atof(&moji[2]);
		}else if( !strncmp("FF", moji, 2) ){
			RG[5] = atof(&moji[2]);
		}else if( !strncmp("DC", moji, 2) ){
			RG[6] = atof(&moji[2]);
		}else if( !strncmp("WD", moji, 2) ){
			RG[7] = atof(&moji[2]);
		}else if( !strncmp("DI", moji, 2) ){
			RG[8] = atof(&moji[2]);
		}else if( !strncmp("AP", moji, 2) ){
			RG[9] = atof(&moji[2]);
		}else if( !strncmp("AN", moji, 2) ){
			RG[10] = atof(&moji[2]);
		}else {
			//printf("Nazo:%s\n",moji);
		}
		++iter;
	}
}


void TinyInterface::VGParam(list<string> strList_VG, float VG[])
{
	list<string>::iterator iter = strList_VG.begin(); 
	while( iter != strList_VG.end() ){
		char moji[100];
		sprintf(moji,"%s",(*iter).c_str());
		if( !strncmp("Tread", moji, 5) ) {
			VG[0] = atof(&moji[5]);
		}else if( !strncmp("Xac", moji, 3) ){
			VG[1] = atof(&moji[3]);
		}else if( !strncmp("Xde", moji, 3) ){
			VG[2] = atof(&moji[3]);
		}else if( !strncmp("Rac", moji, 3) ){
			VG[3] = atof(&moji[3]);
		}else if( !strncmp("Rde", moji, 3) ){
			VG[4] = atof(&moji[3]);
		}else if( !strncmp("Vmax", moji, 4) ){
			VG[5] = atof(&moji[4]);
		}else if( !strncmp("Vmin", moji, 4) ){
			VG[6] = atof(&moji[4]);
		}else if( !strncmp("Rmax", moji, 4) ){
			VG[7] = atof(&moji[4]);
		}else if( !strncmp("Rmin", moji, 4) ){
			VG[8] = atof(&moji[4]);
		}else {
			//printf("Nazo:%s\n",moji);
		}
		++iter;
	}
}

void TinyInterface::recvParam(float RG1[], float RG2[], float VG[])
{
	const int size_RG1 = 11;
	const int size_RG2 = 11;
	const int size_VG = 9;
	string reply_RG1, reply_RG2, reply_VG;
	readData("RG1\n\r", reply_RG1);
	readData("RG2\n\r", reply_RG2);
	readData("VG\n\r", reply_VG);
	printf("RG1:%s\n",reply_RG1.c_str());
	printf("RG2:%s\n",reply_RG2.c_str());
	printf("VG:%s\n",reply_VG.c_str());
    list<string> strList_RG1 = split(reply_RG1, "\n\r>$, :");
    list<string> strList_RG2 = split(reply_RG2, "\n\r>$, :");
    list<string> strList_VG = split(reply_VG, "\n\r>$, :");
    RGParam(strList_RG1, RG1);
    RGParam(strList_RG2, RG2);
    VGParam(strList_VG, VG);
}
