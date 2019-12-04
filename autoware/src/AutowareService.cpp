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
namespace asio = boost::asio;
using asio::ip::tcp;

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
	mReceiverFromCaService = new CommunicationReceiver("23456", "CAM", *mLogger);
	// mLogger = new LoggingUtility("AutowareService", mGlobalConfig.mExpNo, loggingConf, statisticConf);
	mLogger->logStats("speed (m/sec)");
	
	mThreadReceiveFromCaService = new boost::thread(&AutowareService::receiveFromCaService, this);
	// mThreadTestSender = new boost::thread(&AutowareService::testSender, this);

	char cur_dir[1024];
	getcwd(cur_dir, 1024);
	time_t t = time(nullptr);
	const tm* lt = localtime(&t);
	std::stringstream s;
	s<<"20";
	s<<lt->tm_year-100; //100を引くことで20xxのxxの部分になる
	s<<"-";
	s<<lt->tm_mon+1; //月を0からカウントしているため
	s<<"-";
	s<<lt->tm_mday; //そのまま
	s<<"_";
	s<<lt->tm_hour;
	s<<":";
	s<<lt->tm_min;
	s<<":";
	s<<lt->tm_sec;
	std::string timestamp = s.str();

	std::string filename = std::string(cur_dir) + "/../../../autoware/output/delay/" + timestamp + ".csv";
	delay_output_file.open(filename, std::ios::out);

	while(1){
		testSender();
		sleep(1);
	}
}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	delete mReceiverFromCaService;
	delete mThreadReceive;
	delete mThreadReceiveFromCaService;
	delete mThreadTestSender;

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
	for(int i=0; i < s_message.speed.size(); i++){
		autowarePackage::AUTOWARE autoware;
		if(i == s_message.speed.size()-1){
			autoware.set_id(0);
		} else {
			std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
			std::uniform_int_distribution<> rand10000(1, 9999);        // [0, 9999] 範囲の一様乱数
			autoware.set_id(rand10000(mt));
		}
		//write current speed to protocol buffer
		autoware.set_speed(s_message.speed[i]); // standard expects speed in 0.01 m/s
		// autoware.set_time(Utils::currentTime());
		autoware.set_time(s_message.time[i]);
		autoware.set_longitude(s_message.longitude[i]);
		autoware.set_latitude(s_message.latitude[i]);
		// std::cout << std::setprecision(20) << "speed:" << autoware.speed() << " time:" << autoware.time() << " longitude:" << autoware.longitude() << " latitude:" << autoware.latitude() << std::endl;
		sendToServices(autoware);
	}
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
	}
	else {				//use simulated Autoware data
	}
	// ros::spin();
}

double AutowareService::calcSpeed(){
}

void AutowareService::timeCalc(){
}

void AutowareService::receiveFromAutoware(){
	std::cout << "*****receive setup" << std::endl;
	int sockfd;
    int client_sockfd;
    struct sockaddr_in addr;
    socklen_t len = sizeof( struct sockaddr_in );
    struct sockaddr_in from_addr;
    char buf[4096];
 
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
		// socket_message s_message;
		std::stringstream ss;
		memset( buf, 0, sizeof( buf ) );
        rsize = recv( client_sockfd, buf, sizeof( buf ), 0 );
		ss << buf;

		boost::archive::text_iarchive archive(ss);
		archive >> s_message;
		std::cout << "received" << std::endl;
		for(int i = 0; i < s_message.latitude.size(); i++){
			std::cout << "lat:" << s_message.latitude[i] << " lon:" << s_message.longitude[i] << " speed:" << s_message.speed[i] << " time:" << s_message.time[i] << std::endl;
		}
		speed = s_message.speed[0];
		longitude = s_message.longitude[0];
		latitude = s_message.latitude[0];
		generationUnixTime = s_message.time[0];
		std::cout << s_message.speed[0] << std::endl;
        if ( rsize == 0 ) {
            break;
        } else if ( rsize == -1 ) {
            perror( "recv" );
        }
		// simulateData();
    }
 
    close( client_sockfd );
    close( sockfd );


	// asio::io_service io_service;
	// tcp::socket socket(io_service);
	// tcp::acceptor acc(io_service, tcp::endpoint(tcp::v4(), 23457));

	// boost::system::error_code error;
	// acc.accept(socket, error);

	// if(error){
	// 	std::cout << "accept failed:" << error.message() << std::endl;
	// } else {
	// 	std::cout << "accept correct!" << std::endl;
	// }
}



void AutowareService::testSender(){
	while(1){
		s_message.speed.clear();
		s_message.latitude.clear();
		s_message.longitude.clear();
		s_message.time.clear();

		for(int i = 0; i < 10; i++){
			std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
			std::uniform_int_distribution<> rand10000(1, 9999);        // [0, 9999] 範囲の一様乱数
			s_message.speed.push_back(rand10000(mt));
			s_message.time.push_back(rand10000(mt));
			s_message.latitude.push_back(rand10000(mt));
			s_message.longitude.push_back(rand10000(mt));
		}
		simulateData();
		sleep(1);
	}
}

void AutowareService::receiveFromCaService(){
	string serializedAutoware;
	camPackage::CAM cam;

	while (1) {
		pair<string, string> received = mReceiverFromCaService->receive();
		std::cout << "receive from caservice" << std::endl;

		serializedAutoware = received.second;
		cam.ParseFromString(serializedAutoware);
		int64_t currTime = Utils::currentTime();
		long genDeltaTime = (long)(currTime/1000000 - 10728504000) % 65536;
		delay_output_file << std::setprecision(20) << cam.header().stationid() << "" << "," << genDeltaTime << std::endl;
	}
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

	return 0;
}
