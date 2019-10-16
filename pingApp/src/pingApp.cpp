#define ELPP_THREAD_SAFE
#define ELPP_NO_DEFAULT_LOG_FILE

#include "caservice.h"
#include <google/protobuf/text_format.h>
#include <unistd.h>
#include <iostream>
#include <ctime>
#include <chrono>
#include <cmath>
#include <string>
#include <common/utility/Utils.h>
#include <common/asn1/per_encoder.h>

using namespace std;

INITIALIZE_EASYLOGGINGPP

PingApp::PingApp(PingAppConfig &config, ptree& configTree){

    try {
		mGlobalConfig.loadConfig(PING_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}

    mConfig = config;
    mLogger = new LoggingUtility(PING_CONFIG_NAME, PING_MODULE_NAME, mGlobalConfig.mLogBasePath, mGlobalConfig.mExpName, mGlobalConfig.mExpNo, configTree);

    mMsgUtils = new MessageUtils(*mLogger);
	mLogger->logStats("Station Id \tPING id \tCreate Time \tReceive Time");

    
    mSenderToDcc = new CommunicationSender("12345", *mLogger);
    mReceiverFromDcc = new CommunicationReceiver("5555", "PING", *mLogger);
}

PingApp::~PingApp(){
    delete mReceiverFromDcc;
    delete mSenderToDcc;

    delete mLogger;

    delete mMsgUtils;
}

void PingApp::send(){
    string serializedData;
	dataPackage::DATA data;

	// Standard compliant CAM
	CAM_t* cam = generateCam();
	vector<uint8_t> encodedCam = mMsgUtils->encodeMessage(&asn_DEF_CAM, cam);
	string strCam(encodedCam.begin(), encodedCam.end());
	mLogger->logDebug("Encoded CAM size: " + to_string(strCam.length()));

	data.set_id(messageID_cam);
	data.set_type(dataPackage::DATA_Type_CAM);
	data.set_priority(dataPackage::DATA_Priority_BE);

	int64_t currTime = Utils::currentTime();
	data.set_createtime(currTime);
	data.set_validuntil(currTime + mConfig.mExpirationTime*1000*1000*1000);
	data.set_content(strCam);

	data.SerializeToString(&serializedData);
	mLogger->logInfo("PING::Send new CAM to DCC\n");

	mSenderToDcc->send("PING", serializedData);	//send serialized DATA to DCC
}

void PingApp::receive(){
    string envelope;		//envelope
	string serializedAsnCam;	//byte string (serialized)
	string serializedProtoCam;

	while (1) {
		pair<string, string> received = mReceiverFromDcc->receive();
		envelope = received.first;
		serializedAsnCam = received.second;			//serialized DATA

		CAM_t* cam = 0;
		int res = mMsgUtils->decodeMessage(&asn_DEF_CAM, (void **)&cam, serializedAsnCam);
		if (res != 0) {
			mLogger->logError("Failed to decode received CAM. Error code: " + to_string(res));
			continue;
		}
		//asn_fprint(stdout, &asn_DEF_CAM, cam);
		// camPackage::CAM camProto = convertAsn1toProtoBuf(cam);
		// camProto.SerializeToString(&serializedProtoCam);

	}
}

int main(int argc, const char* argv[]){

    ptree configTree = load_config_tree();
    PingConfig pingConfig;

    try{
        pingConfig.loadConfig(configTree);
    }
    catch (std::exception &e){
        cerr << "Error while loading /etc/config/openc2x_ping:" << e.what() << endl << flush;
        return EXIT_FAILURE;
    }
    PingApp ping(pingConfig, configTree);

    return EXIT_SUCCESS;
}