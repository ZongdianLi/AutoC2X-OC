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
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std;
namespace asio = boost::asio;
using asio::ip::tcp;

INITIALIZE_EASYLOGGINGPP	

void setData() {
	std::cout << "simulating....." << std::endl;
	
	autowarePackage::AUTOWAREMCM autoware;

	autoware.set_id(s_message.id);
	autoware.set_messagetype(s_message.messagetype);
	autoware.set_time(s_message.time);
	autoware.set_scenario(s_message.scenario);
	autoware.set_targetstationid(s_message.targetstationid);
	autoware.set_collisiondetected(s_message.collisiondetected);
	autoware.set_adviceaccepted(s_message.adviceaccepted);
	autoware.set_scenarioend(s_message.scenarioend);
	its::TrajectoryPoint* startpoint = new its::TrajectoryPoint();
	startpoint->set_deltalat(s_message.startpoint.deltalat);
	startpoint->set_deltalong(s_message.startpoint.deltalong);
	startpoint->set_deltaalt(s_message.startpoint.deltaalt);
	startpoint->set_x(s_message.startpoint.x);
	startpoint->set_y(s_message.startpoint.y);
	startpoint->set_z(s_message.startpoint.z);
	startpoint->set_w(s_message.startpoint.w);
	startpoint->set_sec(s_message.startpoint.sec);
	startpoint->set_nsec(s_message.startpoint.nsec);
	its::TrajectoryPoint* targetpoint = new its::TrajectoryPoint();
	targetpoint->set_deltalat(s_message.targetpoint.deltalat);
	targetpoint->set_deltalong(s_message.targetpoint.deltalong);
	targetpoint->set_deltaalt(s_message.targetpoint.deltaalt);
	targetpoint->set_x(s_message.targetpoint.x);
	targetpoint->set_y(s_message.targetpoint.y);
	targetpoint->set_z(s_message.targetpoint.z);
	targetpoint->set_w(s_message.targetpoint.w);
	targetpoint->set_sec(s_message.targetpoint.sec);
	targetpoint->set_nsec(s_message.targetpoint.nsec);
	autoware.set_allocated_startpoint(startpoint);
	autoware.set_allocated_targetpoint(targetpoint);

	// for (trajectory_point tp : ego_vehicle_trajectory) {
	for (trajectory_point tp : s_message.trajectory) {
		its::TrajectoryPoint* trajectory_point = autoware.add_trajectory();
		trajectory_point->set_deltalat(tp.deltalat);
		trajectory_point->set_deltaalt(tp.deltalong);
		trajectory_point->set_deltalong(tp.deltaalt);
		trajectory_point->set_x(tp.x);
		trajectory_point->set_y(tp.y);
		trajectory_point->set_z(tp.z);
		trajectory_point->set_w(tp.w);
		trajectory_point->set_sec(tp.sec);
		trajectory_point->set_nsec(tp.nsec);
	}
	
	std::cout << autoware.trajectory_size() << std::endl;
	sendToServices(autoware);
}

void sendToServices(autowarePackage::AUTOWAREMCM autoware) {
	string serializedAutoware;
	autoware.SerializeToString(&serializedAutoware);
	mSender->sendData("AUTOWARE", serializedAutoware);
	// delete mSender;
	// delete mLogger;
}

void storePlannedTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
	// trajectoryを受け取ったらego_vehicle_trajecotoryに保存
	std::string message = in_message->string();
	// const char* msg = message.c_str();
	// std::cout << "subscriberCallback(): Message Received: " << message << std::endl;

	rapidjson::Document d;
	if (d.Parse(message.c_str()).HasParseError()) {
		std::cerr << "advertiseServiceCallback(): Error in parsing service request message: " << message << std::endl;
		return;
	}
	
	// メッセージの中身が正しいかどうか判定
	if (!(d["msg"].IsObject()) || !(d["msg"].HasMember("trajectory"))) return;
	
	ego_vehicle_trajectory.clear();

	// ROSトピックによって後で修正
	// std::cout << d["msg"]["trajectory"].Size() << std::endl;
	for (auto& v : d["msg"]["trajectory"].GetArray()) {
		trajectory_point tp;
		tp.deltalat = (int)(v["pose"]["position"]["x"].GetDouble() * pow(10, 3));
		tp.deltalong = (int)(v["pose"]["position"]["y"].GetDouble() * pow(10, 3));
		tp.deltaalt = (int)(v["pose"]["position"]["z"].GetDouble() * pow(10, 3));
		tp.x = (int)(v["pose"]["orientation"]["x"].GetDouble() * pow(10, 9));
		tp.y = (int)(v["pose"]["orientation"]["y"].GetDouble() * pow(10, 9));
		tp.z = (int)(v["pose"]["orientation"]["z"].GetDouble() * pow(10, 9));
		tp.w = (int)(v["pose"]["orientation"]["w"].GetDouble() * pow(10, 9));
		tp.sec = v["time"]["secs"].GetInt();
		tp.nsec = v["time"]["nsecs"].GetInt();
		ego_vehicle_trajectory.push_back(tp);
	}
	std::cout << ego_vehicle_trajectory.size() << std::endl;
}

void receiveScenarioTrigger(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
	// triggerが来たら保存しておいたego_vehicle_trajectoryをmcserviceに返す
	std::string message = in_message->string();
	std::cout << "subscriberCallback(): Message Received: " << message << std::endl;
	rapidjson::Document d;
	d.Parse(message.c_str());
	if (!(d["msg"].HasMember("data"))) return;
	s_message.id = 0;
	s_message.scenario = d["msg"]["data"].GetInt();
	s_message.trajectory = ego_vehicle_trajectory;
	s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_ADVERTISE;
	// s_message.targetstationid = d["msg"]["data"].GetInt();
	setData();
}

void receiveScenarioTriggerEnd(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
	// triggerが来たら保存しておいたego_vehicle_trajectoryをmcserviceに返す
	std::string message = in_message->string();
	std::cout << "subscriberCallback(): Message Received: " << message << std::endl;
	rapidjson::Document d;
	d.Parse(message.c_str());
	if (!(d["msg"].HasMember("data"))) return;
	s_message.id = 0;
	s_message.scenarioend = 1;
	s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_SCENARIO_FINISH;
	// s_message.targetstationid = d["msg"]["data"].GetInt();
	setData();
}

void detectCollision(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
	// 衝突が検知されたらego_vehicle_trajectoryを、検知されなければcollisiondetected=falseをmcserviceに送信
	std::string message = in_message->string().c_str();
	const char* msg = message.c_str();
	rapidjson::Document d;
	d.Parse(msg);
	s_message.id = 0;
	if (d["msg"]["data"]["detected"] == false) {
		s_message.collisiondetected = 0;
	} else {
		s_message.collisiondetected = 1;
		s_message.targetstationid = d["msg"]["data"]["target_stationID"].GetInt();
		s_message.trajectory = ego_vehicle_trajectory;
	}
	s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_COLLISION_DETECTION_RESULT;
	setData();
	rbc.removeClient("collision_detect");
}

void calculatedDesiredTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
	// other_vehicle/desired_trajectoryをmcserviceに送信
	std::string message = in_message->string().c_str();
	const char* msg = message.c_str();
	rapidjson::Document d;
	d.Parse(msg);
	s_message.id = 0;
	s_message.time = 0;
	s_message.targetstationid = d["msg"]["target_stationID"].GetInt();
	const rapidjson::Value& c = d["msg"]["trajectory"].GetArray();
	struct trajectory_point tp;
	for (auto& v : c.GetArray()) {
		tp.deltalat = (int)v["pose"]["position"]["x"].GetDouble() * pow(10, 3);
		tp.deltalong = (int)v["pose"]["position"]["y"].GetDouble() * pow(10, 3);
		tp.deltaalt = (int)v["pose"]["position"]["z"].GetDouble() * pow(10, 3);
		tp.x = (int)v["pose"]["orientation"]["x"].GetDouble() * pow(10, 9);
		tp.y = (int)v["pose"]["orientation"]["y"].GetDouble() * pow(10, 9);
		tp.z = (int)v["pose"]["orientation"]["z"].GetDouble() * pow(10, 9);
		tp.w = (int)v["pose"]["orientation"]["w"].GetDouble() * pow(10, 9);
		tp.sec = v["header"]["stamp"]["secs"].GetInt();
		tp.nsec = v["header"]["stamp"]["nsecs"].GetInt();
		s_message.trajectory.push_back(tp);
		// std::cout << "list value:" << e << std::endl;
	}
	s_message.startpoint.deltalat = (int)d["msg"]["startpoint"]["pose"]["position"]["x"].GetDouble() * pow(10, 3);
	s_message.startpoint.deltalong = (int)d["msg"]["startpoint"]["pose"]["position"]["y"].GetDouble() * pow(10, 3);
	s_message.startpoint.deltaalt = (int)d["msg"]["startpoint"]["pose"]["position"]["z"].GetDouble() * pow(10, 3);
	s_message.startpoint.x = (int)d["msg"]["startpoint"]["pose"]["orientation"]["x"].GetDouble() * pow(10, 9);
	s_message.startpoint.y = (int)d["msg"]["startpoint"]["pose"]["orientation"]["y"].GetDouble() * pow(10, 9);
	s_message.startpoint.z = (int)d["msg"]["startpoint"]["pose"]["orientation"]["z"].GetDouble() * pow(10, 9);
	s_message.startpoint.w = (int)d["msg"]["startpoint"]["pose"]["orientation"]["w"].GetDouble() * pow(10, 9);
	s_message.startpoint.sec = d["msg"]["startpoint"]["header"]["stamp"]["secs"].GetInt();
	s_message.startpoint.nsec = d["msg"]["startpoint"]["header"]["stamp"]["nsecs"].GetInt();
	s_message.targetpoint.deltalat = (int)d["msg"]["targetpoint"]["pose"]["position"]["x"].GetDouble() * pow(10, 3);
	s_message.targetpoint.deltalong = (int)d["msg"]["targetpoint"]["pose"]["position"]["y"].GetDouble() * pow(10, 3);
	s_message.targetpoint.deltaalt = (int)d["msg"]["targetpoint"]["pose"]["position"]["z"].GetDouble() * pow(10, 3);
	s_message.targetpoint.x = (int)d["msg"]["targetpoint"]["pose"]["orientation"]["x"].GetDouble() * pow(10, 9);
	s_message.targetpoint.y = (int)d["msg"]["targetpoint"]["pose"]["orientation"]["y"].GetDouble() * pow(10, 9);
	s_message.targetpoint.z = (int)d["msg"]["targetpoint"]["pose"]["orientation"]["z"].GetDouble() * pow(10, 9);
	s_message.targetpoint.w = (int)d["msg"]["targetpoint"]["pose"]["orientation"]["w"].GetDouble() * pow(10, 9);
	s_message.targetpoint.sec = d["msg"]["targetpoint"]["header"]["stamp"]["secs"].GetInt();
	s_message.targetpoint.nsec = d["msg"]["targetpoint"]["header"]["stamp"]["nsecs"].GetInt();
	s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_CALCULATED_ROUTE;
	setData();
	rbc.removeClient("calculate_trajectory");
}

void validatedDesiredTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
	std::string message = in_message->string().c_str();
	const char* msg = message.c_str();
	rapidjson::Document d;
	d.Parse(msg);
	s_message.id = 0;
	s_message.time = 0;
	s_message.targetstationid = d["msg"]["data"]["target_stationID"].GetInt();
	s_message.adviceaccepted = d["msg"]["data"]["accept"].GetInt();
	s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_VALIDATED_ROUTE;
	setData();
	rbc.removeClient("validate_trajectory");
}

// void receiveTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message) {
// 	std::string message = in_message->string().c_str();
// 	const char* msg = message.c_str();
// 	rapidjson::Document d;
// 	d.Parse(msg);
// 	std::cout << d["msg"]["points"]["pose"]["x"].GetInt() << std::endl;
// }

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

	// mSender = new CommunicationSender("26666", *mLogger);
	mReceiverFromMcService = new CommunicationReceiver("25555", "MCM", *mLogger);
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
	mThreadReceiveFromMcService = new boost::thread(&AutowareService::receiveFromMcService, this);
	
	// testSender(stoi(argv[1]));
	while(1){
		// testSender();
		sleep(1);
	}

}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	delete mReceiverFromMcService;
	delete mThreadReceive;
	delete mThreadReceiveFromMcService;
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

void AutowareService::receiveFromAutoware(){
	rbc.addClient("ego_vehicle_trajectory");
	rbc.addClient("scenario_trigger");
	rbc.addClient("scenario_trigger_end");
	// rbc.addClient("trajectory");
	// rbc.advertise("scenario_trigger", "/scenario_trigger", "std_msgs/String");
	rbc.subscribe("ego_vehicle_trajectory", "/ego_vehicle/planned_trajectory", storePlannedTrajectory);
	rbc.subscribe("scenario_trigger", "/scenario_trigger", receiveScenarioTrigger);
	rbc.subscribe("scenario_trigger_end", "/scenario_trigger/end", receiveScenarioTriggerEnd);
	// rbc.subscribe("trajectory", "/planning/scenario_planning/lane_driving/trajectory", receiveTrajectory);
	while (1) {
	}
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


void AutowareService::testSender(int msgType){
	int l = 0;
	while(1){
		s_message.id = 2;
		switch (msgType) {
			case 0:
				s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_ADVERTISE;
				break;
			case 1:
				s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_COLLISION_DETECTION_RESULT;
				s_message.collisiondetected = 1;
				break;
			case 2:
				s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_CALCULATED_ROUTE;
				s_message.startpoint.deltalat = 1;
				s_message.startpoint.deltalong = 1;
				s_message.startpoint.deltaalt = 1;
				s_message.startpoint.sec = 1;
				s_message.startpoint.x = 1;
				s_message.startpoint.y = 1;
				s_message.startpoint.z = 1;
				s_message.startpoint.w = 1;
				s_message.startpoint.sec = 1;
				s_message.targetpoint.deltalat = 1;
				s_message.targetpoint.deltalong = 1;
				s_message.targetpoint.deltaalt = 1;
				s_message.targetpoint.sec = 1;
				s_message.targetpoint.x = 1;
				s_message.targetpoint.y = 1;
				s_message.targetpoint.z = 1;
				s_message.targetpoint.w = 1;
				s_message.targetpoint.sec = 1;
				break;
			case 3:
				s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_VALIDATED_ROUTE;
				s_message.adviceaccepted = 1;
				break;
			case 4:
				s_message.messagetype = autowarePackage::AUTOWAREMCM_MessageType_SCENARIO_FINISH;
				s_message.scenarioend = 1;
				break;
		}
		
		s_message.time = 0;
		s_message.scenario = 0;
		s_message.targetstationid = 1;
		struct trajectory_point tp;
		s_message.trajectory.clear();

		for (int i=0; i<140; i++) {
			l += 1;
			tp.deltalat = l;
			tp.deltalong = l;
			tp.deltaalt = l;
			tp.sec = l;
			tp.x = l;
			tp.y = l;
			tp.z = l;
			tp.w = l;
			tp.sec = l;
			s_message.trajectory.push_back(tp);
			// std::cout << l << std::endl;
		}
		setData();
		sleep(1);
	}
}

void AutowareService::receiveFromMcService(){
	string serializedAutoware;
	mcmPackage::MCM mcm;

	while (1) {
		pair<string, string> received = mReceiverFromMcService->receive();
		std::cout << "receive from mcservice" << std::endl;

		serializedAutoware = received.second;
		mcm.ParseFromString(serializedAutoware);
		its::McmParameters_ControlFlag controlFlag = mcm.maneuver().mcmparameters().controlflag();
		json msg;
		rapidjson::Document d;
		msg["targetstationid"] = mcm.header().stationid();
		msg["trajectory"] = json::array();
		json tp;
		tp["time"]["secs"] = 0;
		tp["time"]["nsecs"] = 0;
		tp["pose"]["position"]["x"] = 0;
		tp["pose"]["position"]["y"] = 0;
		tp["pose"]["position"]["z"] = 0;
		tp["pose"]["orientation"]["x"] = 0;
		tp["pose"]["orientation"]["y"] = 0;
		tp["pose"]["orientation"]["z"] = 0;
		tp["pose"]["orientation"]["w"] = 0;
		msg["startpoint"] = tp;
		msg["targetpoint"] = tp;
		switch (controlFlag) {
			case its::McmParameters_ControlFlag_INTENTION_REQUEST:
				for (auto& v : mcm.maneuver().mcmparameters().maneuvercontainer().intentionrequestcontainer().plannedtrajectory()) {
					json tp;
					tp["time"]["secs"] = v.sec();
					tp["time"]["nsecs"] = v.nsec();
					tp["pose"]["position"]["x"] = (double)v.deltalat() / pow(10, 3);
					tp["pose"]["position"]["y"] = (double)v.deltalong() / pow(10, 3);
					tp["pose"]["position"]["z"] = (double)v.deltaalt() / pow(10, 3);
					tp["pose"]["orientation"]["x"] = (double)v.x() / pow(10, 9);
					tp["pose"]["orientation"]["y"] = (double)v.y() / pow(10, 9);
					tp["pose"]["orientation"]["z"] = (double)v.z() / pow(10, 9);
					tp["pose"]["orientation"]["w"] = (double)v.w() / pow(10, 9);
					msg["trajectory"].push_back(tp);
				}
				d.Parse(msg.dump().c_str());
				rbc.addClient("collision_detect");
  				rbc.advertise("collision_detect", "/other_vehicle/planned_trajectory/collision_detect", "mcservice_msgs/TrajectoryWithTargetStationId");
				rbc.publish("/other_vehicle/planned_trajectory/collision_detect", d);
				rbc.subscribe("collision_detect", "/collision_detect", detectCollision);
				break;
			case its::McmParameters_ControlFlag_INTENTION_REPLY:
				for (auto& v : mcm.maneuver().mcmparameters().maneuvercontainer().intentionreplycontainer().plannedtrajectory()) {
					json tp;
					tp["time"]["secs"] = v.sec();
					tp["time"]["nsecs"] = v.nsec();
					tp["pose"]["position"]["x"] = (double)v.deltalat() / pow(10, 3);
					tp["pose"]["position"]["y"] = (double)v.deltalong() / pow(10, 3);
					tp["pose"]["position"]["z"] = (double)v.deltaalt() / pow(10, 3);
					tp["pose"]["orientation"]["x"] = (double)v.x() / pow(10, 9);
					tp["pose"]["orientation"]["y"] = (double)v.y() / pow(10, 9);
					tp["pose"]["orientation"]["z"] = (double)v.z() / pow(10, 9);
					tp["pose"]["orientation"]["w"] = (double)v.w() / pow(10, 9);
					msg["trajectory"].push_back(tp);
				}
				d.Parse(msg.dump().c_str());
				rbc.addClient("calculate_trajectory");
				rbc.advertise("calculate_trajectory", "/other_vehicle/planned_trajectory/calculate", "mcservice_msgs/TrajectoryWithTargetStationId");
				rbc.publish("/other_vehicle/planned_trajectory/calculate", d);
				rbc.subscribe("calculate_trajectory", "/other_vehicle/desired_trajectory", calculatedDesiredTrajectory);
				break;
			case its::McmParameters_ControlFlag_PRESCRIPTION:
				{
					json start_tp;
					start_tp["time"]["secs"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().sec();
					start_tp["time"]["nsecs"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().nsec();
					start_tp["pose"]["position"]["x"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().deltalat() / pow(10, 3);
					start_tp["pose"]["position"]["y"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().deltalong() / pow(10, 3);
					start_tp["pose"]["position"]["z"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().deltaalt() / pow(10, 3);
					start_tp["pose"]["orientation"]["x"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().x() / pow(10, 9);
					start_tp["pose"]["orientation"]["y"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().y() / pow(10, 9);
					start_tp["pose"]["orientation"]["z"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().z() / pow(10, 9);
					start_tp["pose"]["orientation"]["w"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().startpoint().w() / pow(10, 9);
					json target_tp;
					target_tp["time"]["secs"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().sec();
					target_tp["time"]["nsecs"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().nsec();
					target_tp["pose"]["position"]["x"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().deltalat() / pow(10, 3);
					target_tp["pose"]["position"]["y"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().deltalong() / pow(10, 3);
					target_tp["pose"]["position"]["z"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().deltaalt() / pow(10, 3);
					target_tp["pose"]["orientation"]["x"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().x() / pow(10, 9);
					target_tp["pose"]["orientation"]["y"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().y() / pow(10, 9);
					target_tp["pose"]["orientation"]["z"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().z() / pow(10, 9);
					target_tp["pose"]["orientation"]["w"] = mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().optionaldescription().targetpoint().w() / pow(10, 9);
					msg["startpoint"] = start_tp;
					msg["targetpoint"] = target_tp;
					for (auto& v : mcm.maneuver().mcmparameters().maneuvercontainer().prescriptioncontainer().desiredtrajectory()) {
						json tp;
						tp["time"]["secs"] = v.sec();
						tp["time"]["nsecs"] = v.nsec();
						tp["pose"]["position"]["x"] = (double)v.deltalat() / pow(10, 3);
						tp["pose"]["position"]["y"] = (double)v.deltalong() / pow(10, 3);
						tp["pose"]["position"]["z"] = (double)v.deltaalt() / pow(10, 3);
						tp["pose"]["orientation"]["x"] = (double)v.x() / pow(10, 9);
						tp["pose"]["orientation"]["y"] = (double)v.y() / pow(10, 9);
						tp["pose"]["orientation"]["z"] = (double)v.z() / pow(10, 9);
						tp["pose"]["orientation"]["w"] = (double)v.w() / pow(10, 9);
						msg["trajectory"].push_back(tp);
					}
					d.Parse(msg.dump().c_str());
					std::cout << msg.dump().c_str() << std::endl;
					rbc.addClient("validate_trajectory");
					rbc.advertise("validate_trajectory", "/ego_vehicle/desired_trajectory", "mcservice_msgs/TrajectoryWithTargetStationId");
					rbc.publish("/ego_vehicle/desired_trajectory", d);
					rbc.subscribe("validate_trajectory", "/accept_desired_trajectory", validatedDesiredTrajectory);
					break;
				}
			case its::McmParameters_ControlFlag_HEARTBEAT:
				break;
			default:
				break;
		}
	}
}

int main(int argc,  char* argv[]) {

	AutowareConfig config;
	GlobalConfig mGlobalConfig;
	try {
		config.loadConfig();
		mGlobalConfig.loadConfig(AUTOWARE_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading config.xml: " << e.what() << endl << flush;
		return EXIT_FAILURE;
	}
	
	ptree pt = load_config_tree();
	mLogger = new LoggingUtility(AUTOWARE_CONFIG_NAME, AUTOWARE_MODULE_NAME, mGlobalConfig.mLogBasePath, mGlobalConfig.mExpName, mGlobalConfig.mExpNo, pt);
	mSender = new CommunicationSender("26666", *mLogger);

	AutowareService autoware(config, argc, argv);

	return 0;
}
