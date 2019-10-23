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
		config.loadConfig(LDM_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}
	ptree pt = load_config_tree();

	mLogger = new LoggingUtility(LDM_CONFIG_NAME, LDM_MODULE_NAME, config.mLogBasePath, config.mExpName, config.mExpNo, pt);

	string moduleName = "pingApp";
	mReceiverFromCa = new CommunicationReceiver("23456", "CAM", *mLogger);
	
}

pingApp::~pingApp() {
	mThreadReceiveFromCa->join();
	// mThreadServer->join();
	delete mThreadReceiveFromCa;

	delete mReceiverFromCa;

}

void pingApp::init() {
	mThreadReceiveFromCa = new boost::thread(&pingApp::receiveFromCa, this);
	// mThreadServer = new boost::thread(&LDM::receiveRequest, this);
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

void pingApp::receiveFromCa() {
	string serializedCam;	//serialized CAM
	camPackage::CAM cam;

	while (1) {
        std::cout << "receive from ca" << std::endl;

		pair<string, string> received = mReceiverFromCa->receive();	//receive
		serializedCam = received.second;
		cam.ParseFromString(serializedCam);

		//printCam(cam);
		//ASSUMPTION: received cam is the newer than all cams that were received before.
		//TODO: OPTIMIZATION: use pointers instead of copying cams.
		camCache[to_string(cam.header().stationid())]=cam;

	}
}


int main(int argc, const char* argv[]) {

	pingApp pingApp;
	pingApp.init();

	return EXIT_SUCCESS;
}
