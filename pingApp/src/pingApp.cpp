#define ELPP_THREAD_SAFE
#define ELPP_NO_DEFAULT_LOG_FILE

#include "pingApp.h"
#include <google/protobuf/text_format.h>
#include <common/buffers/camInfo.pb.h>
#include <common/buffers/CoopAwareness.pb.h>
#include <unistd.h>
#include <iostream>
#include <ctime>
#include <chrono>
#include <cmath>
#include <string>
#include <common/utility/Utils.h>
#include <common/asn1/CAM.h>
#include <common/asn1/per_encoder.h>

using namespace std;

INITIALIZE_EASYLOGGINGPP

PingApp::PingApp(PingConfig &config, ptree& configTree){

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
	data.set_validuntil(currTime + 1*1000*1000*1000);
	data.set_content(strCam);

	data.SerializeToString(&serializedData);
	mLogger->logInfo("PING::Send new CAM to DCC\n");

	mSenderToDcc->send("PING", serializedData);	//send serialized DATA to DCC
}


//generate new CAM with latest gps and obd2 data
CAM_t* PingApp::generateCam() {
	mLogger->logDebug("Generating CAM as per UPER");
	CAM_t* cam = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));
	if (!cam) {
		throw runtime_error("could not allocate CAM_t");
	}
	// ITS pdu header
	cam->header.stationID = mGlobalConfig.mStationID;// mIdCounter; //
	cam->header.messageID = messageID_cam;
	cam->header.protocolVersion = protocolVersion_currentVersion;

    int num_a = 1;
    int num_b = 1;
    int num_c = 1;

	// generation delta time
	int64_t currTime = Utils::currentTime();
    cam->cam.generationDeltaTime = (currTime/1000000 - 10728504000) % 65536;

	// Basic container
	cam->cam.camParameters.basicContainer.stationType = mConfig.mIsRSU ? StationType_roadSideUnit : StationType_passengerCar;

    cam->cam.camParameters.basicContainer.referencePosition.latitude = num_a; // in one-tenth of microdegrees
    cam->cam.camParameters.basicContainer.referencePosition.longitude = num_b; // in one-tenth of microdegrees
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = num_c;
    cam->cam.camParameters.basicContainer.referencePosition.latitude = Latitude_unavailable;
    cam->cam.camParameters.basicContainer.referencePosition.longitude = Longitude_unavailable;
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

	cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 0;
	cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 0;
	cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 0;

	// High frequency container
	// Could be basic vehicle or RSU and have corresponding details
	if(mConfig.mIsRSU) {
		cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_rsuContainerHighFrequency;
		// Optional fields in CAM from RSU
		// cam->cam.camParameters.highFrequencyContainer.choice.rsuContainerHighFrequency.protectedCommunicationZonesRSU
	} else {
		cam->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue = CurvatureValue_unavailable;
		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence = CurvatureConfidence_unavailable;

		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode = CurvatureCalculationMode_unavailable;

		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = DriveDirection_unavailable;

		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = HeadingValue_unavailable;
		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = HeadingConfidence_unavailable;

		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;


		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;

		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = VehicleWidth_unavailable;

		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue = YawRateValue_unavailable;
		cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence = YawRateConfidence_unavailable;

	}


	// Optional
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.accelerationControl->
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.cenDsrcTollingZone->
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.verticalAcceleration
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.steeringWheelAngle
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.performanceClass
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lanePosition->
	//	cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.lateralAcceleration

	// Optional part
	//	cam->cam.camParameters.lowFrequencyContainer->present =
	//	cam->cam.camParameters.specialVehicleContainer->present =

	// Printing the cam structure
	//	asn_fprint(stdout, &asn_DEF_CAM, cam);


    //TODO: Free the allocated structure for cam. Is this enough?
	//asn_DEF_CAM.free_struct(&asn_DEF_CAM, cam, 0);
    return cam;
}

camPackage::CAM PingApp::convertAsn1toProtoBuf(CAM_t* cam) {
	camPackage::CAM camProto;
	// header
	its::ItsPduHeader* header = new its::ItsPduHeader;
	header->set_messageid(cam->header.messageID);
	header->set_protocolversion(cam->header.protocolVersion);
	header->set_stationid(cam->header.stationID);
	camProto.set_allocated_header(header);

	// coop awareness
	its::CoopAwareness* coop = new its::CoopAwareness;
	coop->set_gendeltatime(cam->cam.generationDeltaTime);
	its::CamParameters* params = new its::CamParameters;

	// basic container
	its::BasicContainer* basicContainer = new its::BasicContainer;

	basicContainer->set_stationtype(cam->cam.camParameters.basicContainer.stationType);
	basicContainer->set_latitude(cam->cam.camParameters.basicContainer.referencePosition.latitude);
	basicContainer->set_longitude(cam->cam.camParameters.basicContainer.referencePosition.longitude);
	basicContainer->set_altitude(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue);
	basicContainer->set_altitudeconfidence(cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence);
	basicContainer->set_semimajorconfidence(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence);
	basicContainer->set_semiminorconfidence(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence);
	basicContainer->set_semimajororientation(cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation);
	params->set_allocated_basiccontainer(basicContainer);

	// high frequency container
	its::HighFreqContainer* highFreqContainer = new its::HighFreqContainer;
	its::BasicVehicleHighFreqContainer* basicHighFreqContainer = 0;
	its::RsuHighFreqContainer* rsuHighFreqContainer = 0;
	switch (cam->cam.camParameters.highFrequencyContainer.present) {
		case HighFrequencyContainer_PR_basicVehicleContainerHighFrequency:
			highFreqContainer->set_type(its::HighFreqContainer_Type_BASIC_HIGH_FREQ_CONTAINER);
			basicHighFreqContainer = new its::BasicVehicleHighFreqContainer();
			basicHighFreqContainer->set_heading(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue);
			basicHighFreqContainer->set_headingconfidence(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence);
			basicHighFreqContainer->set_speed(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue);
			basicHighFreqContainer->set_speedconfidence(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence);
			basicHighFreqContainer->set_drivedirection(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection);
			basicHighFreqContainer->set_vehiclelength(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue);
			basicHighFreqContainer->set_vehiclelengthconfidence(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication);
			basicHighFreqContainer->set_vehiclewidth(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth);
			basicHighFreqContainer->set_longitudinalacceleration(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue);
			basicHighFreqContainer->set_longitudinalaccelerationconfidence(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue);
			basicHighFreqContainer->set_curvature(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue);
			basicHighFreqContainer->set_curvatureconfidence(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureConfidence);
			basicHighFreqContainer->set_curvaturecalcmode(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvatureCalculationMode);
			basicHighFreqContainer->set_yawrate(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue);
			basicHighFreqContainer->set_yawrateconfidence(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence);

			// optional fields
			//basicHighFreqContainer->set_accelerationcontrol();
			//basicHighFreqContainer->set_laneposition();
			//basicHighFreqContainer->set_steeringwheelangle();
			//basicHighFreqContainer->set_steeringwheelangleconfidence();
			//basicHighFreqContainer->set_lateralacceleration();
			//basicHighFreqContainer->set_lateralaccelerationconfidence();
			//basicHighFreqContainer->set_verticalacceleration();
			//basicHighFreqContainer->set_verticalaccelerationconfidence();
			//basicHighFreqContainer->set_performanceclass();
			//basicHighFreqContainer->set_protectedzonelatitude();
			//basicHighFreqContainer->set_has_protectedzonelongitude();
			//basicHighFreqContainer->set_cendsrctollingzoneid();

			highFreqContainer->set_allocated_basicvehiclehighfreqcontainer(basicHighFreqContainer);
			break;

		case HighFrequencyContainer_PR_rsuContainerHighFrequency:
			highFreqContainer->set_type(its::HighFreqContainer_Type_RSU_HIGH_FREQ_CONTAINER);

			rsuHighFreqContainer = new its::RsuHighFreqContainer();
			// optional fields
			//rsuHighFreqContainer->

			highFreqContainer->set_allocated_rsuhighfreqcontainer(rsuHighFreqContainer);
			break;

		default:
			break;
	}
	params->set_allocated_highfreqcontainer(highFreqContainer);

	// low frequency container (optional)
	if(!cam->cam.camParameters.lowFrequencyContainer) {
		// fill in the low freq container
	}

	// special vehicle container (optional)
	if(!cam->cam.camParameters.specialVehicleContainer) {
		// fill in the special vehicle container
	}

	coop->set_allocated_camparameters(params);
	camProto.set_allocated_coop(coop);

	return camProto;
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