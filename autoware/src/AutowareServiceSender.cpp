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

#include "AutowareServiceSender.h"
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


AutowareService::AutowareService(AutowareConfig &config, int argc, char* argv[]) {
	flag = -1;
	try {
		mGlobalConfig.loadConfig(AUTOWARE_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}
	mConfig = config;
	ptree pt = load_config_tree();
	mLogger = new LoggingUtility(AUTOWARE_CONFIG_NAME, AUTOWARE_MODULE_NAME, mGlobalConfig.mLogBasePath, mGlobalConfig.mExpName, mGlobalConfig.mExpNo, pt);

	mSender = new CommunicationSender("26666", *mLogger);
	mReceiverFromCaService = new CommunicationReceiver("23456", "CAM", *mLogger);
	mLogger->logStats("speed (m/sec)");

	loadOpt(argc, argv);
	
	//autowareとの1対1の時だけファイルアウトプットはコメントアウトする
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

	mThreadReceive = new boost::thread(&AutowareService::receiveFromAutoware, this);
	mThreadReceiveFromCaService = new boost::thread(&AutowareService::receiveFromCaService, this);
	
	testSender();
	while(1){
		// testSender();
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

}

void AutowareService::loadOpt(int argc, char* argv[]){	
	int i, opt;
	opterr = 0; //getopt()のエラーメッセージを無効にする。
    //オプション以外の引数を出力する
    for (i = optind; i < argc; i++) {
		host_addr = std::string(argv[i]);
		std::cout << host_addr << std::endl;
		break;
    }
	if(host_addr.length() < 4){
		printf("Usage: %s  [host_addr] ...\n", argv[0]);
	}
}


//simulates Autoware data, logs and sends it
void AutowareService::setData() {
	std::cout << "simulating....." << std::endl;
	
	autowarePackage::AUTOWAREMCM autoware;

	autoware.set_id(s_message.id);
	autoware.set_time(s_message.time);
	autoware.set_scenerio(s_message.scenerio);
	autoware.set_targetstationid(s_message.targetstationid);

	// autoware.clear_trajectory();
	for (int i=0; i<s_message.trajectory.size(); i++) {
		its::TrajectoryPoint* trajectory_point = autoware.add_trajectory();
		trajectory_point->set_deltalat(s_message.trajectory[i].deltalat);
		trajectory_point->set_deltaalt(s_message.trajectory[i].deltalong);
		trajectory_point->set_deltalong(s_message.trajectory[i].deltaalt);
		trajectory_point->set_pathdeltatime(s_message.trajectory[i].pathdeltatime);
	}
	
	std::cout << autoware.trajectory_size() << std::endl;
	// for (int i=0; i<autoware.trajectory_size(); i++) {
	// 	std::cout << autoware.trajectory(i).deltalat() << std::endl;
	// }

	sendToServices(autoware);

	// for(unsigned int i=0; i < s_message.speed.size(); i++){
	// 	autowarePackage::AUTOWARE autoware;
	// 	std::cout << "stationid is :::" << s_message.stationid[i] << std::endl;
	// 	autoware.set_id(s_message.stationid[i]);
	// 	autoware.set_speed(s_message.speed[i]); // standard expects speed in 0.01 m/s
	// 	autoware.set_time(s_message.time[i]);
	// 	autoware.set_longitude(s_message.longitude[i]);
	// 	autoware.set_latitude(s_message.latitude[i]);
	// 	sendToServices(autoware);
	// 	if(s_message.stationid[i] == 0){
	// 		latitude = s_message.latitude[i];
	// 		longitude = s_message.longitude[i];
	// 	}
	// }
}


//logs and sends Autoware
void AutowareService::sendToServices(autowarePackage::AUTOWAREMCM autoware) {
	//send buffer to services
	string serializedAutoware;
	autoware.SerializeToString(&serializedAutoware);
	mSender->sendData("AUTOWARE", serializedAutoware);
	// mLogger->logStats(to_string(autoware.speed()) + " (" + to_string(autoware.speed()/100*3.6) + "km/h)"); // In csv, we log speed in m/sec
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
		std::stringstream ss;
		memset( buf, 0, sizeof( buf ) );
        rsize = recv( client_sockfd, buf, sizeof( buf ), 0 );
		
		ss << buf;
		boost::archive::text_iarchive archive(ss);
		archive >> s_message;

		std::cout << "received" << std::endl;
	
        if ( rsize == 0 ) {
            break;	
        } else if ( rsize == -1 ) {
            perror( "recv" );
        }
		std::cout << s_message.timestamp << std::endl;
		setData();
		sendBackToAutoware(s_message);
    }
 
    close( client_sockfd );
    close( sockfd );

}

void AutowareService::sendBackToAutoware(socket_message msg){
	if(flag != 100){
		std::cout << "socket open" << std::endl;
		struct sockaddr_in addr;
		if(( sock_fd = socket(AF_INET, SOCK_STREAM, 0) ) < 0 ) perror("socket");
		addr.sin_family = AF_INET;
		addr.sin_port = htons(23458);
		addr.sin_addr.s_addr = inet_addr(host_addr.c_str());
		connect( sock_fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in) );
		flag = 100;
	}
	
	std::cout << "reflecting:::" << ""<<  msg.timestamp << std::endl;
	std::stringstream ss;
	boost::archive::text_oarchive archive(ss);
	archive << msg;

	ss.seekg(0, ios::end);
	if( send( sock_fd, ss.str().c_str(), ss.tellp(), 0) < 0 ){
		perror("send");
	} else {
	}
}


void AutowareService::testSender(){
	while(1){
		s_message.id = 0;
		s_message.time = 0;
		s_message.scenerio = 0;
		s_message.targetstationid = 1;
		struct trajectory_point tp;
		s_message.trajectory.clear();

		for (int i=0; i<10; i++) {
			tp.deltalat = i;
			tp.deltalong = i;
			tp.deltaalt = i;
			tp.pathdeltatime = i;
			s_message.trajectory.push_back(tp);
		}

		// s_message.speed.clear();
		// s_message.adviceaccepted.clear();
		// s_message.latitude.clear();
		// s_message.longitude.clear();
		// s_message.time.clear();
		// s_message.stationid.clear();

		// s_message.stationid.push_back(100);
		// s_message.speed.push_back(1919);
		// s_message.latitude.push_back(35.714464 * 10000000);
		// s_message.longitude.push_back(139.760606 * 10000000);
		// s_message.time.push_back(1919);

		// s_message.stationid.push_back(101);
		// s_message.speed.push_back(1919);
		// s_message.latitude.push_back(35.71419722 * 10000000);
		// s_message.longitude.push_back(139.76148888 * 10000000);
		// s_message.time.push_back(1919);

		// s_message.stationid.push_back(102);
		// s_message.speed.push_back(1919);
		// s_message.latitude.push_back(35.714497 * 10000000);
		// s_message.longitude.push_back(139.763014 * 10000000);
		// s_message.time.push_back(1919);

		// s_message.stationid.push_back(103);
		// s_message.speed.push_back(1919);
		// s_message.latitude.push_back(35.713997 * 10000000);
		// s_message.longitude.push_back(139.760153 * 10000000);
		// s_message.time.push_back(1919);

		// s_message.stationid.push_back(104);
		// s_message.speed.push_back(1919);
		// s_message.latitude.push_back(35.712992 * 10000000);
		// s_message.longitude.push_back(139.759819 * 10000000);
		// s_message.time.push_back(1919);
		setData();
		sleep(1);
	}
}

void AutowareService::receiveFromCaService(){
	// string serializedAutoware;
	// camPackage::CAM cam;

	// while (1) {
	// 	pair<string, string> received = mReceiverFromCaService->receive();
	// 	std::cout << "receive from caservice" << std::endl;

	// 	serializedAutoware = received.second;
	// 	cam.ParseFromString(serializedAutoware);
	// 	int64_t currTime = Utils::currentTime();
	// 	long genDeltaTime = (long)(currTime/1000000 - 10728504000) % 65536;
	// 	delay_output_file << std::setprecision(20) << cam.header().stationid() << "" << "," << genDeltaTime << "," << currTime << "," << latitude << "," << longitude << std::endl;

	// 	socket_message msg;
	// 	msg.timestamp = 10;
	// 	msg.speed.push_back(0);
	// 	msg.latitude.push_back(0);
	// 	msg.longitude.push_back(0);
	// 	msg.time.push_back(0);
	// 	msg.stationid.push_back(cam.header().stationid());
	// }
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
	AutowareService autoware(config, argc, argv);

	return 0;
}
