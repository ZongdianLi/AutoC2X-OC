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

#include "mcserviceSender.h"
#include <google/protobuf/text_format.h>
#include <unistd.h>
#include <iostream>
#include <ctime>
#include <chrono>
#include <cmath>
#include <string>
#include <common/utility/Utils.h>
#include <common/asn1/per_encoder.h>
#include <random>

using namespace std;

INITIALIZE_EASYLOGGINGPP

McService::McService(McServiceConfig &config, ptree& configTree, char* argv[]) {
	switch (stoi(argv[1])) {
		case 0:
			state = Waiting;
			break;
		case 1:
			state = Advertising;
			break;
		case 2:
			state = CollisionDetecting;
			break;
		case 3:
			state = Prescripting;
			break;
		case 4:
			state = NegotiatingPrescriber;
			break;
		case 5:
			state = NegotiatingReceiver;
			break;
		case 6:
			state = ActivatingPrescriber;
			break;
		case 7:
			state = ActivatingReceiver;
			break;
		case 8:
			state = Finishing;
			break;
		case 9:
			state = Abending;
			break;
	}
	std::cout << "state: " << state << std::endl;
	
	try {
		mGlobalConfig.loadConfig(MCM_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}

	std::cout << mGlobalConfig.mStationID << std::endl;

	mConfig = config;
	mLogger = new LoggingUtility(MCM_CONFIG_NAME, MCM_MODULE_NAME, mGlobalConfig.mLogBasePath, mGlobalConfig.mExpName, mGlobalConfig.mExpNo, configTree);

	mMsgUtils = new MessageUtils(*mLogger);
	mLogger->logStats("Station Id \tMCM id \tCreate Time \tReceive Time");

	mReceiverFromDcc = new CommunicationReceiver("5555", "MCM", *mLogger);
	mSenderToDcc = new CommunicationSender("23333", *mLogger);
	mSenderToLdm = new CommunicationSender("24444", *mLogger);
	mSenderToAutoware = new CommunicationSender("25555", *mLogger);

	// mReceiverGps = new CommunicationReceiver( "3333", "GPS", *mLogger);
	// mReceiverObd2 = new CommunicationReceiver("2222", "OBD2", *mLogger);
	mReceiverAutoware = new CommunicationReceiver("26666", "AUTOWARE",*mLogger);

	mThreadReceive = new boost::thread(&McService::receive, this);
	// mThreadGpsDataReceive = new boost::thread(&McService::receiveGpsData, this);
	// mThreadObd2DataReceive = new boost::thread(&McService::receiveObd2Data, this);
	mThreadAutowareDataReceive = new boost::thread(&McService::receiveAutowareData, this);
	

	mIdCounter = 0;

	mGpsValid = false;	//initially no data is available => not valid
	mObd2Valid = false;
	// mAutowareValid = false;
	mAutowareValid = true;
}

McService::~McService() {
	mThreadReceive->join();
	// mThreadGpsDataReceive->join();
	// mThreadObd2DataReceive->join();
	mThreadAutowareDataReceive->join();

	delete mThreadReceive;
	// delete mThreadGpsDataReceive;
	// delete mThreadObd2DataReceive;
	delete mThreadAutowareDataReceive;

	delete mReceiverFromDcc;
	delete mSenderToDcc;
	delete mSenderToLdm;

	// delete mReceiverGps;
	// delete mReceiverObd2;
	delete mReceiverAutoware;

	delete mLogger;

	delete mMsgUtils;

	mTimer->cancel();
	delete mTimer;
}

//receive MCM from DCC and forward to LDM and autoware?
void McService::receive() {
	string envelope;		//envelope
	string serializedAsnMcm;	//byte string (serialized)
	string serializedProtoMcm;

	while (1) {
		pair<string, string> received = mReceiverFromDcc->receive();
		std::cout << " ****** okaeri ******" << std::endl;
		envelope = received.first;
		serializedAsnMcm = received.second;			//serialized DATA

		MCM_t* mcm = 0;
		int res = mMsgUtils->decodeMessage(&asn_DEF_MCM, (void **)&mcm, serializedAsnMcm);
		if (res != 0) {
			mLogger->logError("Failed to decode received MCM. Error code: " + to_string(res));
			continue;
		}
		//asn_fprint(stdout, &asn_DEF_MCM, mcm);
		mcmPackage::MCM mcmProto = convertAsn1toProtoBuf(mcm);

		if ((mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_CANCEL || mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ABEND) && mcmProto.maneuver().mcmparameters().maneuvercontainer().intentionreplycontainer().targetstationid() == mGlobalConfig.mStationID) {
			// ToDo: 普通の自動運転に戻るための処理
			state = Waiting;
			continue;
		}

		switch (state) {
			case Waiting:
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_INTENTION_REQUEST) {
					std::cout << mcmProto.maneuver().mcmparameters().maneuvercontainer().intentionrequestcontainer().plannedtrajectory_size() << std::endl;
					mcmProto.SerializeToString(&serializedProtoMcm);
					mSenderToAutoware->send(envelope, serializedProtoMcm);
					state = CollisionDetecting;
				} else {
				}
				break;
			case Advertising:
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_INTENTION_REPLY && mcmProto.maneuver().mcmparameters().maneuvercontainer().intentionreplycontainer().targetstationid() == mGlobalConfig.mStationID) {
					std::cout << "aaaa" << std::endl;
					mcmProto.SerializeToString(&serializedProtoMcm);
					mSenderToAutoware->send(envelope, serializedProtoMcm);
					state = Prescripting;
					mLatestAutoware.set_targetstationid(mcmProto.header().stationid());
					send(true, Ack);
				} else {
				}
				break;
			case Prescripting:
				std::cout << mcmProto.maneuver().mcmparameters().maneuvercontainer().ackcontainer().targetstationid() << std::endl;
				std::cout << mGlobalConfig.mStationID << std::endl;
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ACK && mcmProto.maneuver().mcmparameters().maneuvercontainer().ackcontainer().targetstationid() == mGlobalConfig.mStationID) {
					ack = true;
					mcmProto.SerializeToString(&serializedProtoMcm);
					mSenderToAutoware->send(envelope, serializedProtoMcm);
					state = NegotiatingPrescriber;
				} else {
				}
				break;
			case NegotiatingPrescriber:
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ACCEPTANCE && mcmProto.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().targetstationid() == mGlobalConfig.mStationID) {
					if (mcmProto.maneuver().mcmparameters().maneuvercontainer().acceptancecontainer().adviceaccepted() == 1) {
						state = ActivatingPrescriber;
						mLatestAutoware.set_targetstationid(mcmProto.header().stationid());
						send(true, Ack);
					} else {
						mcmProto.SerializeToString(&serializedProtoMcm);
						mSenderToAutoware->send(envelope, serializedProtoMcm);
						state = Prescripting;
					}
				} else if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ACK && mcmProto.maneuver().mcmparameters().maneuvercontainer().ackcontainer().targetstationid() == mGlobalConfig.mStationID) {
					ack = true;
				}
				break;
			case NegotiatingReceiver:
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_PRESCRIPTION && mcmProto.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().targetstationid() == mGlobalConfig.mStationID) {
					mcmProto.SerializeToString(&serializedProtoMcm);
					mSenderToAutoware->send(envelope, serializedProtoMcm);
					mLatestAutoware.set_targetstationid(mcmProto.header().stationid());
					send(true, Ack);
				} else if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ACK && mcmProto.maneuver().mcmparameters().maneuvercontainer().ackcontainer().targetstationid() == mGlobalConfig.mStationID) {
					ack = true;
				}
				break;
			case ActivatingPrescriber:
				break;
			case ActivatingReceiver:
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_HEARTBEAT && mcmProto.maneuver().mcmparameters().maneuvercontainer().heartbeatcontainer().targetstationid() == mGlobalConfig.mStationID) {
					mcmProto.SerializeToString(&serializedProtoMcm);
					mSenderToAutoware->send(envelope, serializedProtoMcm);
				} else if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_FIN) {
					state = Waiting;
					mLatestAutoware.set_targetstationid(mcmProto.header().stationid());
					send(true, Ack);
				} else if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ACK && mcmProto.maneuver().mcmparameters().maneuvercontainer().ackcontainer().targetstationid() == mGlobalConfig.mStationID) {
					ack = true;
				}
				break;
			case Finishing:
				if (mcmProto.maneuver().mcmparameters().controlflag() == its::McmParameters_ControlFlag_ACK && mcmProto.maneuver().mcmparameters().maneuvercontainer().ackcontainer().targetstationid() == mGlobalConfig.mStationID) {
					mcmProto.SerializeToString(&serializedProtoMcm);
					mSenderToAutoware->send(envelope, serializedProtoMcm);
					ack = true;
					state = Waiting;
				}
			case Abending:
				break;			
			default:
				break;
		}
		// mcmProto.SerializeToString(&serializedProtoMcm);

		// mLogger->logInfo("Forward incoming MCM " + to_string(mcm->header.stationID) + " to LDM");
		// mSenderToLdm->send(envelope, serializedProtoMcm);	//send serialized MCM to LDM //帰ってきたMCMは今回はLDMには格納しない

		// mSenderToAutoware->send(envelope, serializedProtoMcm);
	}
}

void McService::receiveAutowareData() { //実装
	string serializedAutoware;
	autowarePackage::AUTOWAREMCM newAutoware;

	while (1) {
		serializedAutoware = mReceiverAutoware->receiveData();
		waiting_data.ParseFromString(serializedAutoware);
		std::cout << "----------" << std::endl;
		std::cout << waiting_data.trajectory_size() << std::endl;
		for (int i=0; i<waiting_data.trajectory_size(); i++) {
			its::TrajectoryPoint tp = waiting_data.trajectory(i);
			std::cout << tp.deltalat() << std::endl;
		}
		std::cout << "----------" << std::endl;
		switch (state) {
			case Waiting:
				if (waiting_data.messagetype() == autowarePackage::AUTOWAREMCM_MessageType_ADVERTISE) {
					state = Advertising;
					mLatestAutoware = waiting_data;
					std::cout << "receive advertise" << std::endl;
					trigger(IntentionRequest, 100);
				}
				break;
			case CollisionDetecting:
				if (waiting_data.messagetype() == autowarePackage::AUTOWAREMCM_MessageType_COLLISION_DETECTION_RESULT) {
					if (waiting_data.collisiondetected() == 1) {
						mLatestAutoware = waiting_data;
						state = NegotiatingReceiver;
						ack = false;
						trigger(IntentionReply, 100);
					} else {
						state = Waiting;
					}
				}
			case Advertising:
				break;
			case Prescripting:
				if (waiting_data.messagetype() == autowarePackage::AUTOWAREMCM_MessageType_CALCULATED_ROUTE) {
					mLatestAutoware = waiting_data;
					ack = false;
					trigger(Prescription, 100);
				}
				break;
			case NegotiatingPrescriber:
				break;
			case NegotiatingReceiver:
				if (waiting_data.messagetype() == autowarePackage::AUTOWAREMCM_MessageType_VALIDATED_ROUTE) {
					if (waiting_data.adviceaccepted() == 1) state = ActivatingReceiver;
					mLatestAutoware = waiting_data;
					ack = false;
					trigger(Acceptance, 100);
				}
				break;
			case ActivatingPrescriber:
				if (waiting_data.messagetype() == autowarePackage::AUTOWAREMCM_MessageType_SCENARIO_FINISH) {
					state = Finishing;
					mLatestAutoware = waiting_data;
					ack = false;
					trigger(Fin, 100);
				}
				break;
			case ActivatingReceiver:
			case Finishing:
			case Abending:
			default:
				break;
		}
		//mLogger->logDebug("Received AUTOWARE with speed (m/s): " + to_string(10));
		//mMutexLatestAutoware.lock();
		//mLatestAutoware = newAutoware;
		//mMutexLatestAutoware.lock();
		// if(newAutoware.id() == 0 && waiting_data.size() == 0){
		// 	waiting_data.clear();
		// 	std::cout << "clear because 0" << std::endl;
		// 	//waiting_data.shrink_to_fit();
		// }
		// waiting_data.push_back(newAutoware);
		//mMutexLatestAutoware.unlock();
		// atoc_delay_output_file << Utils::currentTime() << "," << (Utils::currentTime() - newAutoware.time()) / 1000000.0 << std::endl;
	}
}

//sends info about triggering to LDM
void McService::sendMcmInfo(string triggerReason, double delta) {
	infoPackage::McmInfo mcmInfo;
	string serializedMcmInfo;

	mcmInfo.set_time(Utils::currentTime());
	mcmInfo.set_triggerreason(triggerReason);
	mcmInfo.set_delta(delta);

	mcmInfo.SerializeToString(&serializedMcmInfo);
	mSenderToLdm->send("mcmInfo", serializedMcmInfo);
}

//periodically check generation rules for sending to LDM and DCC
void McService::alarm(const boost::system::error_code &ec, Type type) {
	// Check heading and position conditions only if we have valid GPS data
	std::cout << "alarm" << std::endl;
	std::cout << "type: " << type << std::endl;
	std::cout << "state: " << state << std::endl;

	mTimer->cancel();
	delete mTimer;

	switch (type) {
		case IntentionRequest:
			if (state == Advertising) {
				trigger(type, 1000);
			}
		case IntentionReply:
		case Prescription:
		case Acceptance:
		case Fin:
			if (!ack) {
				trigger(type, 1000);
			}
			break;
		case Heartbeat:
			if (state == ActivatingReceiver) {
				trigger(type, 1000);
			}
			break;
		default:
			break;
	}
}

void McService::trigger(Type type, int interval) {
	std::cout << "trigger" << std::endl;
	send(true, type);
	scheduleNextAlarm(type, interval);
}

bool McService::isTimeToTriggerMCM() {
	//max. time interval 1s
	int64_t currentTime = Utils::currentTime();
	int64_t deltaTime = currentTime - mLastSentMcmInfo.timestamp;
	if(deltaTime >= 1*100*1000*1000) {
		// sendMcmInfo("time", deltaTime);
		//mLogger->logInfo("deltaTime: " + to_string(deltaTime));
		return true;
	}
	return false;
}

void McService::scheduleNextAlarm(Type type, int interval) {
	//min. time interval 0.1 s
	mTimer = new boost::asio::deadline_timer(mIoService, boost::posix_time::millisec(interval));
	mTimer->async_wait(boost::bind(&McService::alarm, this, boost::asio::placeholders::error, type));
	mIoService.run();
	std::cout << "schedule next alarm" << std::endl;
}

//generate MCM and send to LDM and DCC
void McService::send(bool isAutoware, Type type) {
	std::chrono::system_clock::time_point start, end;
	start = std::chrono::system_clock::now();

	std::cout << "*********lets send MCM:" << std::endl;
	for (int i=0; i<mLatestAutoware.trajectory_size(); i++) {
		its::TrajectoryPoint tp = mLatestAutoware.trajectory(i);
		std::cout << tp.deltalat() << std::endl;
	}

	// std::cout << "send****" << std::endl;
	string serializedData;
	dataPackage::DATA data;

	// Standard compliant MCM
	MCM_t* mcm = generateMcm(isAutoware, type);
	
	char error_buffer[128];
	size_t error_length = sizeof(error_buffer);
	const int return_value = asn_check_constraints(&asn_DEF_MCM, mcm, error_buffer, &error_length); // validate the message
	if (return_value) std::cout << error_buffer << std::endl;
	
	vector<uint8_t> encodedMcm = mMsgUtils->encodeMessage(&asn_DEF_MCM, mcm);
	string strMcm(encodedMcm.begin(), encodedMcm.end());
	//mLogger->logDebug("Encoded MCM size: " + to_string(strMcm.length()));

	// data.set_id(messageID_mcm);
	data.set_id(messageID_mcm);
	data.set_type(dataPackage::DATA_Type_MCM);
	data.set_priority(dataPackage::DATA_Priority_BE);

	int64_t currTime = Utils::currentTime();
	data.set_createtime(currTime);
	data.set_validuntil(currTime + mConfig.mExpirationTime*1000*1000*1000);
	data.set_content(strMcm);

	data.SerializeToString(&serializedData);
	//mLogger->logInfo("Send new MCM to DCC and LDM\n");

	mSenderToDcc->send("MCM", serializedData);	//send serialized DATA to DCC

	mcmPackage::MCM mcmProto = convertAsn1toProtoBuf(mcm);
	string serializedProtoMcm;
	mcmProto.SerializeToString(&serializedProtoMcm);
	mSenderToLdm->send("MCM", serializedProtoMcm); //send serialized MCM to LDM
	asn_DEF_MCM.free_struct(&asn_DEF_MCM, mcm, 0);
}

//generate new MCM with latest gps and obd2 data
MCM_t* McService::generateMcm(bool isAutoware, Type type) {
	//mLogger->logDebug("Generating MCM as per UPER");
	MCM_t* mcm = static_cast<MCM_t*>(calloc(1, sizeof(MCM_t)));
	if (!mcm) {
		throw runtime_error("could not allocate MCM_t");
	}
	// ITS pdu header
	// if (isAutoware){
	// 	mcm->header.stationID = mLatestAutoware.id();
	// } else {
	mcm->header.stationID = mGlobalConfig.mStationID;// mIdCounter; //
	// }
	mcm->header.messageID = messageID_mcm;
	mcm->header.protocolVersion = protocolVersion_currentVersion;

	// generation delta time
	int64_t currTime = Utils::currentTime();
	if (false){
		// mcm->mcm.generationDeltaTime = mLatestPingApp.time();
	} else {
		if (mLastSentMcmInfo.timestamp) {
			// mcm->mcm.generationDeltaTime = (currTime - mLastSentMcmInfo.timestamp) / (100000260);
			mcm->mcm.generationDeltaTime = (currTime/1000000 - 10728504000) % 65536;
		} else {
			mcm->mcm.generationDeltaTime = 0;
		}
	}
	mLastSentMcmInfo.timestamp = currTime;

	// Basic container
	mcm->mcm.mcmParameters.basicContainer.stationType = mConfig.mIsRSU ? StationType_roadSideUnit : StationType_passengerCar;

	mMutexLatestGps.lock();
	
	mLastSentMcmInfo.hasGPS = false;
	mcm->mcm.mcmParameters.basicContainer.referencePosition.latitude = Latitude_unavailable;
	mcm->mcm.mcmParameters.basicContainer.referencePosition.longitude = Longitude_unavailable;
	mcm->mcm.mcmParameters.basicContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	mcm->mcm.mcmParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	mMutexLatestGps.unlock();

	mcm->mcm.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 0;
	mcm->mcm.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 0;
	mcm->mcm.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 0;

	// Maneuver Container

	Trajectory_t* trajectory = static_cast<Trajectory_t*>(calloc(1, sizeof(Trajectory_t)));
		
	for (int i=0; i<mLatestAutoware.trajectory_size(); i++) {
		TrajectoryPoint_t* trajectory_point = static_cast<TrajectoryPoint_t*>(calloc(1, sizeof(TrajectoryPoint_t)));
		if (trajectory_point == NULL) {
			perror("calloc() failed");
			exit(EXIT_FAILURE);
		}
		trajectory_point->pathPosition.deltaLatitude = mLatestAutoware.trajectory(i).deltalat();
		trajectory_point->pathPosition.deltaLongitude = mLatestAutoware.trajectory(i).deltalong();
		trajectory_point->pathPosition.deltaAltitude = mLatestAutoware.trajectory(i).deltaalt();
		trajectory_point->pathDeltaTime = mLatestAutoware.trajectory(i).pathdeltatime();
		const int result = asn_sequence_add(trajectory, trajectory_point);
	}

	if (mAutowareValid) {
		switch(type) {
			case IntentionRequest:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_intentionRequestContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_intentionRequest;
				mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.scenario = mLatestAutoware.scenario();
				mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.plannedTrajectory = *trajectory;
				break;
			case IntentionReply:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_intentionReplyContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_intentionReply;
				mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.targetStationID = mLatestAutoware.targetstationid();
				mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.plannedTrajectory = *trajectory;
				break;
			case Prescription:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_prescriptionContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_prescription;
				mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.targetStationID = mLatestAutoware.targetstationid();
				mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.desiredTrajectory = *trajectory;
				break;
			case Acceptance:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_acceptanceContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_acceptance;
				mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.targetStationID = mLatestAutoware.targetstationid();
				mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.adviceAccepted = mLatestAutoware.adviceaccepted();
				mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.selectedTrajectory = *trajectory;
				break;
			case Heartbeat:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_heartbeatContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_heartbeat;
				mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.targetStationID = mLatestAutoware.targetstationid();
				mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.selectedTrajectory = *trajectory;
				break;
			case Ack:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_ackContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_ack;
				mcm->mcm.mcmParameters.maneuverContainer.choice.ackContainer.targetStationID = mLatestAutoware.targetstationid();
				break;
			case Fin:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_finContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_fin;
				mcm->mcm.mcmParameters.maneuverContainer.choice.finContainer.targetStationID = mLatestAutoware.targetstationid();
				break;
			case Cancel:
				mcm->mcm.mcmParameters.maneuverContainer.present = ManeuverContainer_PR_cancelContainer;
				mcm->mcm.mcmParameters.controlFlag = controlFlag_cancel;
				mcm->mcm.mcmParameters.maneuverContainer.choice.cancelContainer.targetStationID = mLatestAutoware.targetstationid();
				break;
			default:
				break;
		}
	}

    return mcm;
}

mcmPackage::MCM McService::convertAsn1toProtoBuf(MCM_t* mcm) {
	mcmPackage::MCM mcmProto;
	// header
	its::ItsPduHeader* header = new its::ItsPduHeader;
	header->set_messageid(mcm->header.messageID);
	header->set_protocolversion(mcm->header.protocolVersion);
	header->set_stationid(mcm->header.stationID);
	mcmProto.set_allocated_header(header);

	// coop awareness
	its::ManeuverCoordination* maneuver = new its::ManeuverCoordination;
	maneuver->set_gendeltatime(mcm->mcm.generationDeltaTime);
	its::McmParameters* params = new its::McmParameters;

	// basic container
	its::BasicContainer* basicContainer = new its::BasicContainer;

	basicContainer->set_stationtype(mcm->mcm.mcmParameters.basicContainer.stationType);
	basicContainer->set_latitude(mcm->mcm.mcmParameters.basicContainer.referencePosition.latitude);
	basicContainer->set_longitude(mcm->mcm.mcmParameters.basicContainer.referencePosition.longitude);
	basicContainer->set_altitude(mcm->mcm.mcmParameters.basicContainer.referencePosition.altitude.altitudeValue);
	basicContainer->set_altitudeconfidence(mcm->mcm.mcmParameters.basicContainer.referencePosition.altitude.altitudeConfidence);
	basicContainer->set_semimajorconfidence(mcm->mcm.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence);
	basicContainer->set_semiminorconfidence(mcm->mcm.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence);
	basicContainer->set_semimajororientation(mcm->mcm.mcmParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation);
	params->set_allocated_basiccontainer(basicContainer);

	// maneuver container
	its::ManeuverContainer* maneuverContainer = new its::ManeuverContainer;
	its::IntentionRequestContainer* intentionRequestContainer = 0;
	its::IntentionReplyContainer* intentionReplyContainer = 0;
	its::PrescriptionContainer* prescriptionContainer = 0;
	its::AcceptanceContainer* acceptanceContainer = 0;
	its::HeartbeatContainer* heartbeatContainer = 0;
	its::AckContainer* ackContainer = 0;
	its::FinContainer* finContainer = 0;
	its::CancelContainer* cancelContainer = 0;

	switch (mcm->mcm.mcmParameters.maneuverContainer.present) {
		case ManeuverContainer_PR_intentionRequestContainer:
 			// maneuverContainer->set_type(its::ManeuverContainer_Type_INTENTION_REQUEST);
			params->set_controlflag(its::McmParameters_ControlFlag_INTENTION_REQUEST);
			intentionRequestContainer = new its::IntentionRequestContainer();
			intentionRequestContainer->set_scenario(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.scenario);
			for (int i=0; i<mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.plannedTrajectory.list.count; i++) {
				its::TrajectoryPoint* trajectory_point = intentionRequestContainer->add_plannedtrajectory();
				trajectory_point->set_deltalat(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.plannedTrajectory.list.array[i]->pathPosition.deltaLatitude);
				trajectory_point->set_deltaalt(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.plannedTrajectory.list.array[i]->pathPosition.deltaAltitude);
				trajectory_point->set_deltalong(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.plannedTrajectory.list.array[i]->pathPosition.deltaLongitude);
				trajectory_point->set_pathdeltatime(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionRequestContainer.plannedTrajectory.list.array[i]->pathDeltaTime);
			}
			maneuverContainer->set_allocated_intentionrequestcontainer(intentionRequestContainer);
			break;
		case ManeuverContainer_PR_intentionReplyContainer:
			// maneuverContainer->set_type(its::ManeuverContainer_Type_INTENTION_REPLY);
			params->set_controlflag(its::McmParameters_ControlFlag_INTENTION_REPLY);
			intentionReplyContainer = new its::IntentionReplyContainer();
			intentionReplyContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.targetStationID);
			for (int i=0; i<mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.plannedTrajectory.list.count; i++) {
				its::TrajectoryPoint* trajectory_point = intentionReplyContainer->add_plannedtrajectory();
				trajectory_point->set_deltalat(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.plannedTrajectory.list.array[i]->pathPosition.deltaLatitude);
				trajectory_point->set_deltaalt(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.plannedTrajectory.list.array[i]->pathPosition.deltaAltitude);
				trajectory_point->set_deltalong(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.plannedTrajectory.list.array[i]->pathPosition.deltaLongitude);
				trajectory_point->set_pathdeltatime(mcm->mcm.mcmParameters.maneuverContainer.choice.intentionReplyContainer.plannedTrajectory.list.array[i]->pathDeltaTime);
			}
			maneuverContainer->set_allocated_intentionreplycontainer(intentionReplyContainer);
			break;
		case ManeuverContainer_PR_prescriptionContainer:
			// maneuverContainer->set_type(its::ManeuverContainer_Type_PRESCRIPTION);
			params->set_controlflag(its::McmParameters_ControlFlag_PRESCRIPTION);
			prescriptionContainer = new its::PrescriptionContainer();
			prescriptionContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.targetStationID);
			for (int i=0; i<mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.desiredTrajectory.list.count; i++) {
				its::TrajectoryPoint* trajectory_point = prescriptionContainer->add_desiredtrajectory();
				trajectory_point->set_deltalat(mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.desiredTrajectory.list.array[i]->pathPosition.deltaLatitude);
				trajectory_point->set_deltaalt(mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.desiredTrajectory.list.array[i]->pathPosition.deltaAltitude);
				trajectory_point->set_deltalong(mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.desiredTrajectory.list.array[i]->pathPosition.deltaLongitude);
				trajectory_point->set_pathdeltatime(mcm->mcm.mcmParameters.maneuverContainer.choice.prescriptionContainer.desiredTrajectory.list.array[i]->pathDeltaTime);
			}
			maneuverContainer->set_allocated_prescriptioncontainer(prescriptionContainer);
			break;
		case ManeuverContainer_PR_acceptanceContainer:
			// maneuverContainer->set_type(its::ManeuverContainer_Type_ACCEPTANCE);
			params->set_controlflag(its::McmParameters_ControlFlag_ACCEPTANCE);
			acceptanceContainer = new its::AcceptanceContainer();
			acceptanceContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.targetStationID);
			acceptanceContainer->set_adviceaccepted(mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.adviceAccepted);
			for (int i=0; i<mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.selectedTrajectory.list.count; i++) {
				its::TrajectoryPoint* trajectory_point = acceptanceContainer->add_selectedtrajectory();
				trajectory_point->set_deltalat(mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.selectedTrajectory.list.array[i]->pathPosition.deltaLatitude);
				trajectory_point->set_deltaalt(mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.selectedTrajectory.list.array[i]->pathPosition.deltaAltitude);
				trajectory_point->set_deltalong(mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.selectedTrajectory.list.array[i]->pathPosition.deltaLongitude);
				trajectory_point->set_pathdeltatime(mcm->mcm.mcmParameters.maneuverContainer.choice.acceptanceContainer.selectedTrajectory.list.array[i]->pathDeltaTime);
			}
			maneuverContainer->set_allocated_acceptancecontainer(acceptanceContainer);
			break;
		case ManeuverContainer_PR_heartbeatContainer:
			// maneuverContainer->set_type(its::ManeuverContainer_Type_HEARTBEAT);
			params->set_controlflag(its::McmParameters_ControlFlag_HEARTBEAT);
			heartbeatContainer = new its::HeartbeatContainer();
			heartbeatContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.targetStationID);
			for (int i=0; i<mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.selectedTrajectory.list.count; i++) {
				its::TrajectoryPoint* trajectory_point = heartbeatContainer->add_selectedtrajectory();
				trajectory_point->set_deltalat(mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.selectedTrajectory.list.array[i]->pathPosition.deltaLatitude);
				trajectory_point->set_deltaalt(mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.selectedTrajectory.list.array[i]->pathPosition.deltaAltitude);
				trajectory_point->set_deltalong(mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.selectedTrajectory.list.array[i]->pathPosition.deltaLongitude);
				trajectory_point->set_pathdeltatime(mcm->mcm.mcmParameters.maneuverContainer.choice.heartbeatContainer.selectedTrajectory.list.array[i]->pathDeltaTime);
			}
			maneuverContainer->set_allocated_heartbeatcontainer(heartbeatContainer);
			break;
		case ManeuverContainer_PR_ackContainer:
			// maneuverContainer->set_type(its::ManeuverContainer_Type_ACK);
			params->set_controlflag(its::McmParameters_ControlFlag_ACK);
			ackContainer = new its::AckContainer();
			ackContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.ackContainer.targetStationID);
			maneuverContainer->set_allocated_ackcontainer(ackContainer);
			break;
		case ManeuverContainer_PR_finContainer:
			// maneuverContainer->set_type(its::ManeuverContainer_Type_FIN);
			params->set_controlflag(its::McmParameters_ControlFlag_FIN);
			std::cout << params->controlflag() << std::endl;
			finContainer = new its::FinContainer();
			finContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.finContainer.targetStationID);
			std::cout << finContainer->targetstationid() << std::endl;
			maneuverContainer->set_allocated_fincontainer(finContainer);
			break;
		case ManeuverContainer_PR_cancelContainer:
			params->set_controlflag(its::McmParameters_ControlFlag_CANCEL);
			cancelContainer = new its::CancelContainer();
			cancelContainer->set_targetstationid(mcm->mcm.mcmParameters.maneuverContainer.choice.cancelContainer.targetStationID);
			maneuverContainer->set_allocated_cancelcontainer(cancelContainer);
			break;
		default:
			break;
	}

	params->set_allocated_maneuvercontainer(maneuverContainer);

	maneuver->set_allocated_mcmparameters(params);
	mcmProto.set_allocated_maneuver(maneuver);

	return mcmProto;
}

int main(int argc, char* argv[]) {
	ptree configTree = load_config_tree();
	McServiceConfig mcConfig;
	try {
		mcConfig.loadConfig(configTree);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_mcm: " << e.what() << endl << flush;
		return EXIT_FAILURE;
	}
	McService mcm(mcConfig, configTree, argv);

	return EXIT_SUCCESS;
}
