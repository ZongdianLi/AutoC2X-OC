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

	mReceiverFromCa = new CommunicationReceiver("23456", "CAM", *mLogger);
	
	//for simulation only
	mRandNumberGen = default_random_engine(0);
	mBernoulli = bernoulli_distribution(0);
	mUniform = uniform_real_distribution<double>(-0.01, 0.01);

	mThreadReceive = new boost::thread(&AutowareService::receiveFromCa, this);
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

	struct sockaddr_in addr;
	if( (sockfd = socket( AF_INET, SOCK_STREAM, 0) ) < 0 ) perror( "socket" ); 
	addr.sin_family = AF_INET;
	addr.sin_port = htons( 23457 );
	addr.sin_addr.s_addr = inet_addr( "192.168.1.1" );
	connect( sockfd, (struct sockaddr *)&addr, sizeof( struct sockaddr_in ) );

}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	mTimer->cancel();
	delete mTimer;
}

//reads the actual vehicle data from Autoware
void AutowareService::receiveData(const boost::system::error_code &ec, SerialPort* serial) {
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
	// sendToServices(autoware);

	// mTimer->expires_from_now(boost::posix_time::millisec(mConfig.mFrequency));
	// mTimer->async_wait(boost::bind(&AutowareService::simulateData, this, boost::asio::placeholders::error));
}

//logs and sends Autoware
void AutowareService::receiveFromCa() {
	string serializedCam;
	camPackage::CAM cam;

	while(1){
		pair<string, string> received = mReceiverFromCa->receive();
		std::cout << "receive from ca" << std::endl;

		serializedCam = received.second;
		cam.ParseFromString(serializedCam);
		std::cout << "stationid:" << cam.header().stationid() << " latitude:" << cam.coop().camparameters().basiccontainer().latitude() << " longitude:" << cam.coop().camparameters().basiccontainer().longitude() << " speed:" <<  cam.coop().camparameters().highfreqcontainer().basicvehiclehighfreqcontainer().speed() << std::endl;

		s_message.latitude.push_back( cam.coop().camparameters().basiccontainer().latitude() );
		s_message.longitude.push_back( cam.coop().camparameters().basiccontainer().longitude() );
		s_message.time.push_back( cam.coop().gendeltatime() );
		s_message.speed.push_back( cam.coop().camparameters().highfreqcontainer().basicvehiclehighfreqcontainer().speed() );
	}
}


void AutowareService::init() {
	if (!mConfig.mSimulateData) {	//use real Autoware data
	}
	else {				//use simulated Autoware data
	}
}

double AutowareService::calcSpeed(){
}

void AutowareService::timeCalc(){
}

void AutowareService::sendToAutoware(){
	char send_str[10];
    char receive_str[10];
	message message;
	message.speed = 100;
	message.time =  (114) % 65536;
	message.longitude = 35.713968752011098218 * 10000000;
	message.latitude = 139.76268463349990157 * 10000000;
	char* my_s_bytes = static_cast<char*>(static_cast<void*>(&message));
	if( send( sockfd, my_s_bytes, sizeof(message), 0 ) < 0 ) {
			perror( "send" );
	} else {
	}
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
