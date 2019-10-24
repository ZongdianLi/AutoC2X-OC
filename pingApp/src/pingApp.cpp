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

#include "pingApp.h"
#include <unistd.h>
#include <iostream>
#include <common/config/config.h>
#include <map>
#include <common/utility/Utils.h>

using namespace std;

INITIALIZE_EASYLOGGINGPP


pingApp::pingApp() {
	GlobalConfig config;
	try {
		config.loadConfig(CAM_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}
	ptree pt = load_config_tree();

	mLogger = new LoggingUtility(CAM_CONFIG_NAME, CAM_CONFIG_NAME, config.mLogBasePath, config.mExpName, config.mExpNo, pt);

	string moduleName = "pingApp";
	mReceiverFromCa = new CommunicationReceiver("23456", "CAM", *mLogger);
	
    mSender = new CommunicationSender("34567", *mLogger);

	if(config.mStationID == 1){
		mThreadSender = new boost::thread(&pingApp::sendInit, this);
	}
	// mTimer = new boost::asio::deadline_timer(mIoService, boost::posix_time::millisec(100));
    // mTimer->async_wait(boost::bind(&pingApp::alarm, this, boost::asio::placeholders::error));
    // mIoService.run();

}

pingApp::~pingApp() {
	mThreadReceiveFromCa->join();
	delete mThreadReceiveFromCa;
	delete mReceiverFromCa;
	delete mThreadSender;
    delete mSender;
}

void pingApp::sendInit(){
	pingAppPackage::PINGAPP pingApp;
	while(true){
		int64_t currTime = Utils::currentTime();
		pingApp.set_time((currTime/1000000 - 10728504000) % 65536);
		int r = rnd() % 600000;
		pingApp.set_latitude(r);
		pingApp.set_stationid(99991);
		camCache[r] = (currTime/1000000 - 10728504000) % 65536;
		sendToCaService(pingApp);
		std::cout << "send lat:" << r  << " timestamp:" << (currTime/1000000 - 10728504000) % 65536 << std::endl;
		sleep(1);
	}
}

void pingApp::init() {
	std::cout << "init*****" << std::endl;
	mThreadReceiveFromCa = new boost::thread(&pingApp::receiveFromCa, this);
}


void pingApp::receiveFromCa() {
	string serializedCam;	//serialized CAM
	camPackage::CAM cam;
	while (1) {
		pair<string, string> received = mReceiverFromCa->receive();	//receive
		serializedCam = received.second;
		cam.ParseFromString(serializedCam);
        // std::cout << "receive from ca id:" << cam.header().stationid() << std::endl;


		if(cam.header().stationid() == 99991){ //return ping
			pingAppPackage::PINGAPP pingApp;
			pingApp.set_time(cam.coop().gendeltatime());
			pingApp.set_latitude(cam.coop().camparameters().basiccontainer().latitude());
			pingApp.set_stationid(99992);
			sendToCaService(pingApp);
		}

		if(cam.header().stationid() == 99992){ //return ping
			int64_t currTime = Utils::currentTime();
			// std::cout << "receive lat:" << cam.coop().camparameters().basiccontainer().latitude() << " nowTime:" <<  (currTime/1000000 - 10728504000) % 65536 << std::endl;
			// std::cout << "delay:" << (currTime/1000000 - 10728504000) % 65536 - camCache[cam.coop().camparameters().basiccontainer().latitude()] << std::endl;
		}

		// std::cout << "stationID:" << cam.header().stationid() << std::endl;
		// std::cout << "gendeltatime:" << cam.coop().gendeltatime() << std::endl;

		//printCam(cam);
		//ASSUMPTION: received cam is the newer than all cams that were received before.
		//TODO: OPTIMIZATION: use pointers instead of copying cams.
		// camCache[to_string(cam.header().stationid())]=cam;

	}
}

void pingApp::sendToCaService(pingAppPackage::PINGAPP pingApp){
    //send buffer to services
	string serializedPingApp;
    std::cout << "send to caservice " << pingApp.time() << std::endl;
	pingApp.SerializeToString(&serializedPingApp);
	mSender->sendData("PINGAPP", serializedPingApp);
	//log position
	// string csvPosition = to_string(gps.latitude()) + "\t" + to_string(gps.longitude()) + "\t" + to_string(gps.altitude());
	// mLogger->logStats(csvPosition);
}

void pingApp::alarm(const boost::system::error_code &ec){
    pingAppPackage::PINGAPP pingApp;
    int64_t currTime = Utils::currentTime();
    pingApp.set_time((currTime/1000000 - 10728504000) % 65536);
    sendToCaService(pingApp);
    scheduleNextAlarm();
}

void pingApp::scheduleNextAlarm(){
    mTimer->expires_from_now(boost::posix_time::millisec(1000));
	mTimer->async_wait(boost::bind(&pingApp::alarm, this, boost::asio::placeholders::error));
}


int main(int argc, const char* argv[]) {

	pingApp pingApp;
	pingApp.init();

	return EXIT_SUCCESS;
}




// void LDM::receiveRequest() {
// 	string envelope, request, reply;
// 	mLogger->logDebug("waiting for request");
// 	while(1) {
// 		pair<string, string> received = mServer->receiveRequest();
// 		envelope = received.first;	//specifies table
// 		request = received.second;	//specifies condition

// 		if (envelope.compare("CAM") == 0) {
// 			dataPackage::LdmData cams = camSelect(request);
// 			cams.SerializeToString(&reply);
// 		}
// 		else {
// 			reply = request;
// 		}

// 		mServer->sendReply(reply);
// 	}
// }
