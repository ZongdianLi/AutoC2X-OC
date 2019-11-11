// This file is part of OpenC2X.
//
// OpenC2X is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// OpenC2X is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with OpenC2X.  If not, see <http://www.gnu.org/licenses/>.
//
// Authors:
// Sven Laux <slaux@mail.uni-paderborn.de>
// Gurjashan Singh Pannu <gurjashan.pannu@ccs-labs.org>
// Stefan Schneider <stefan.schneider@ccs-labs.org>
// Jan Tiemann <janhentie@web.de>


#define ELPP_THREAD_SAFE
#define ELPP_NO_DEFAULT_LOG_FILE

#include "AutowareService.h"
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <common/utility/Utils.h>
#include <math.h>

using namespace std;

INITIALIZE_EASYLOGGINGPP


AutowareService::AutowareService(AutowareConfig &config) {
	try {
		mGlobalConfig.loadConfig(AUTOWARE_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}
	mConfig = config;
	ptree pt = load_config_tree();
	mLogger = new LoggingUtility(AUTOWARE_CONFIG_NAME, AUTOWARE_MODULE_NAME, mGlobalConfig.mLogBasePath, mGlobalConfig.mExpName, mGlobalConfig.mExpNo, pt);

	mSender = new CommunicationSender("25000", *mLogger);
	// mLogger = new LoggingUtility("AutowareService", mGlobalConfig.mExpNo, loggingConf, statisticConf);
	mLogger->logStats("speed (m/sec)");
	
	//for simulation only
	mRandNumberGen = default_random_engine(0);
	mBernoulli = bernoulli_distribution(0);
	mUniform = uniform_real_distribution<double>(-0.01, 0.01);

	mThreadReceive = new boost::thread(&AutowareService::receiveFromAutoware, this);
	// mThreadReceive = new boost::thread(&AutowareService::testSender, this);
	// testSender();
	

	// char cur_dir[1024];
	// getcwd(cur_dir, 1024);
	// time_t t = time(nullptr);
	// const tm* lt = localtime(&t);
	// std::stringstream s;
	// s<<"20";
	// s<<lt->tm_year-100; //100を引くことで20xxのxxの部分になる
	// s<<"-";
	// s<<lt->tm_mon+1; //月を0からカウントしているため
	// s<<"-";
	// s<<lt->tm_mday; //そのまま
	// s<<"_";
	// s<<lt->tm_hour;
	// s<<":";
	// s<<lt->tm_min;
	// s<<":";
	// s<<lt->tm_sec;
	// std::string timestamp = s.str();

	// std::string filename = std::string(cur_dir) + "/../../../autoware/output/delay/" + timestamp + ".csv";
	// delay_output_file.open(filename, std::ios::out);

}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	mTimer->cancel();
	delete mTimer;
}

//reads the actual vehicle data from Autoware
void AutowareService::receiveData(const boost::system::error_code &ec, SerialPort* serial) {
	// double speed = serial->readSpeed();
	// int rpm = serial->readRpm();
	// cout << to_string(speed) << " " << to_string(rpm) << endl;
	// if (speed != -1) {		//valid speed
	// 	//write current data to protocol buffer
	// 	autowarePackage::AUTOWARE autoware;
	// 	autoware.set_time(Utils::currentTime());
	// 	autoware.set_speed(speed * 100); // standard expects speed in 0.01 m/s
	// 	if (rpm != -1) {
	// 		autoware.set_rpm(rpm);
	// 	}
	// 	sendToServices(autoware);
	// }
	// mTimer->expires_from_now(boost::posix_time::millisec(mConfig.mFrequency));
	// mTimer->async_wait(boost::bind(&AutowareService::receiveData, this, boost::asio::placeholders::error, serial));
}



//simulates realistic vehicle speed
void AutowareService::simulateSpeed() {
	double pCurr = mBernoulli.p();
	double pNew = pCurr + mUniform(mRandNumberGen);
	pNew = min(0.15, max(0.0, pNew));		//pNew always between 0 and 0.15

	mBernoulli = bernoulli_distribution(pNew);

	double sum = 0;

	for (int i=0; i<1000; i++) {
		double r = mBernoulli(mRandNumberGen);
		sum += min(1.0, max(0.0, r)) * 100;	//*100 to convert to 0-15 m/s
	}
	cout << to_string(sum) << endl;
	speed = sum / 1000.0;
	// return sum / 1000.0;					//avg to avoid rapid/drastic changes in speed
}

//simulates Autoware data, logs and sends it
void AutowareService::simulateData() {
	std::cout << "simulating....." << std::endl;
	autowarePackage::AUTOWARE autoware;

	//write current speed to protocol buffer
	autoware.set_speed(speed); // standard expects speed in 0.01 m/s
	// autoware.set_time(Utils::currentTime());
	autoware.set_time(generationUnixTime);
	autoware.set_longitude(longitude);
	autoware.set_latitude(latitude);
	std::cout << std::setprecision(20) << "speed:" << autoware.speed() << " time:" << autoware.time() << " longitude:" << autoware.longitude() << " latitude:" << autoware.latitude() << std::endl;
	sendToServices(autoware);

	// mTimer->expires_from_now(boost::posix_time::millisec(mConfig.mFrequency));
	// mTimer->async_wait(boost::bind(&AutowareService::simulateData, this, boost::asio::placeholders::error));
}

//logs and sends Autoware
void AutowareService::sendToServices(autowarePackage::AUTOWARE autoware) {
	//send buffer to services
	string serializedAutoware;
	autoware.SerializeToString(&serializedAutoware);
	mSender->sendData("AUTOWARE", serializedAutoware);
	mLogger->logStats(to_string(autoware.speed()) + " (" + to_string(autoware.speed()/100*3.6) + "km/h)"); // In csv, we log speed in m/sec
}


void AutowareService::init() {
	if (!mConfig.mSimulateData) {	//use real Autoware data
		// SerialPort* serial = new SerialPort();
		// if (serial->connect(mConfig.mDevice) != -1) {
		// 	mLogger->logInfo("Connected to serial port successfully");

		// 	serial->init();

		// 	mTimer = new boost::asio::deadline_timer(mIoService, boost::posix_time::millisec(mConfig.mFrequency));
		// 	mTimer->async_wait(boost::bind(&AutowareService::receiveData, this, boost::asio::placeholders::error, serial));
		// 	mIoService.run();

		// 	serial->disconnect();
		// }
		// else {
		// 	mLogger->logError("Cannot open serial port -> plug in Autoware and run with sudo");
		// }
	}
	else {				//use simulated Autoware data
		// mTimer = new boost::asio::deadline_timer(mIoService, boost::posix_time::millisec(mConfig.mFrequency));
		// mTimer->async_wait(boost::bind(&AutowareService::simulateData, this, boost::asio::placeholders::error));
		// mIoService.run();
	}
	// ros::spin();

}

double AutowareService::calcSpeed(){
}

void AutowareService::timeCalc(){
	// geometry_msgs::PoseStamped newestPose = nowPose;
	// double messageRosTime = newestPose.header.stamp.sec +  newestPose.header.stamp.nsec / 1000000000.0;
	// double diffTimeFromRosToWall = (ros::WallTime::now().toSec() - ros::Time::now().toSec() - ros::Time::now().toSec() + ros::WallTime::now().toSec()) / 2.0;
	// std::cout << "delay:" << (ros::WallTime::now().toSec() - (messageRosTime + diffTimeFromRosToWall)) *1000 << std::endl;
	// delay_output_file << ros::WallTime::now() << "," << (ros::WallTime::now().toSec() - (messageRosTime + diffTimeFromRosToWall)) *1000 << std::endl;
	// generationUnixTime = messageRosTime + diffTimeFromRosToWall;
}

void AutowareService::receiveFromAutoware(){
	std::cout << "*****receive setup" << std::endl;
	int sockfd;
    int client_sockfd;
    struct sockaddr_in addr;
    socklen_t len = sizeof( struct sockaddr_in );
    struct sockaddr_in from_addr;
    char buf[1024];
 
    memset( buf, 0, sizeof( buf ) );
    if( ( sockfd = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) {
        perror( "socket" );
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons( 23457 );
    addr.sin_addr.s_addr = INADDR_ANY;
    if( bind( sockfd, (struct sockaddr *)&addr, sizeof( addr ) ) < 0 ) perror( "bind" );
    if( listen( sockfd, SOMAXCONN ) < 0 ) perror( "listen" );
    if( ( client_sockfd = accept( sockfd, (struct sockaddr *)&from_addr, &len ) ) < 0 ) perror( "accept" );
 
    // 受信
    int rsize;
    while( 1 ) {
		message message;
        rsize = recv( client_sockfd, buf, sizeof( buf ), 0 );

		memcpy(&message, buf, sizeof(message));
		speed = message.speed;
		longitude = message.longitude;
		latitude = message.latitude;
		generationUnixTime = message.time;
		std::cout << message.speed << std::endl;
        if ( rsize == 0 ) {
            break;
        } else if ( rsize == -1 ) {
            perror( "recv" );
        }
		simulateData();
    }
 
    close( client_sockfd );
    close( sockfd );
}

void AutowareService::testSender(){
	int sockfd;
    struct sockaddr_in addr;
    if( (sockfd = socket( AF_INET, SOCK_STREAM, 0) ) < 0 ) perror( "socket" ); 
    addr.sin_family = AF_INET;
    addr.sin_port = htons( 23457 );
    addr.sin_addr.s_addr = inet_addr( "127.0.0.1" );
    connect( sockfd, (struct sockaddr *)&addr, sizeof( struct sockaddr_in ) );
 
    // データ送信
    char send_str[10];
    char receive_str[10];
	message message;
    for ( int i = 0; i < 10; i++ ){
        sprintf( send_str, "%d", i );
		message.speed = i + 10000;
		message.time = i * 2;
		message.longitude = i * 5;
		message.latitude = i * 3;
		char* my_s_bytes = static_cast<char*>(static_cast<void*>(&message));
        if( send( sockfd, my_s_bytes, sizeof(message), 0 ) < 0 ) {
            perror( "send" );
        } else {
        }
        sleep( 1 );
    }
    close( sockfd );
}

int main(int argc,  char* argv[]) {

	AutowareConfig config;
	try {
		config.loadConfig();
	}
	catch (std::exception &e) {
		cerr << "Error while loading config.xml: " << e.what() << endl << flush;
		return EXIT_FAILURE;
	}
	AutowareService autoware(config);

	while(1){
		sleep(100);
	}

	return 0;
}
