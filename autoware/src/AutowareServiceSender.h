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


#ifndef AUTOWARESERVICE_H_
#define AUTOWARESERVICE_H_

/**
 * @addtogroup autoware
 * @{
 */

#include <common/utility/CommunicationSender.h>
#include <common/utility/CommunicationReceiver.h>
#include <common/utility/LoggingUtility.h>
#include <common/utility/Constants.h>
#include <common/buffers/cam.pb.h>
#include <common/buffers/mcm.pb.h>
#include <common/buffers/autoware.pb.h>
#include <common/buffers/autowareMcm.pb.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <string>
#include <common/config/config.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <random>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "rosbridge_ws_client.hpp"

using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;

/** Struct that holds the configuration for AutowareService.
 * The configuration is defined in <a href="../../autoware/config/config.xml">autoware/config/config.xml</a>.
 */
struct AutowareConfig {
	/** True iff AUTOWARE should be simulated, false for using real data.
	 *
	 */
	bool mSimulateData;

	/** The USB device AUTOWARE is connected to (default: /dev/ttyUSB0).
	 *
	 */
	char* mDevice;

	/** The frequency in ms at which new AUTOWARE data should be distributed (default: 500).
	 *	Note: a frequency of 100ms does not work.
	 */
	int mFrequency;

	// void loadConfigXML(const std::string &filename) {
	// 	boost::property_tree::ptree pt;
	// 	read_xml(filename, pt);

	// 	mSimulateData = pt.get("autoware.SimulateData", true);
	// 	std::string device = pt.get("autoware.Device", "//dev//ttyUSB0");
	// 	mDevice = strdup(device.c_str());
	// 	mFrequency = pt.get("autoware.Frequency", 100);
	// }

	void loadConfig() {
		
		ptree pt = load_config_tree();
		
		mSimulateData = pt.get("autoware.simulateData", true);
		std::string device = pt.get("autoware.device", "//dev//ttyUSB0");
		mDevice = strdup(device.c_str());
		mFrequency = pt.get("autoware.frequency", 100);

		
	}
};

RosbridgeWsClient rbc("localhost:9090");

std::vector<struct trajectory_point> ego_vehicle_trajectory;
std::vector<struct trajectory_point> other_vehicle_trajectory;

CommunicationSender* mSender;
LoggingUtility* mLogger;

struct trajectory_point{
	int deltalat;
	int deltalong;
	int deltaalt;
	int x;
	int y;
	int z;
	int w;
	int sec;
	int nsec;

private:
	friend class boost::serialization::access;
	template<class Archive>
		void serialize( Archive& ar, unsigned int ver){
			ar & deltalat;
			ar & deltalong;
			ar & deltaalt;
			ar & x;
			ar & y;
			ar & z;
			ar & w;
			ar & sec;
			ar & nsec;
		}
};

struct socket_message{
	long timestamp;
	int id;
	autowarePackage::AUTOWAREMCM_MessageType messagetype;
	int time;
	int scenario;
	int targetstationid;
	std::vector<struct trajectory_point> trajectory;
	int collisiondetected;
	int adviceaccepted;
	int scenarioend;

private:
	friend class boost::serialization::access;
	template<class Archive>
		void serialize( Archive& ar, unsigned int ver){
			ar & timestamp;
			ar & id;
			ar & messagetype;
			ar & time;
			ar & scenario;
			ar & targetstationid;
			ar & trajectory;
			ar & collisiondetected;
			ar & adviceaccepted;
			ar & scenarioend;
		}
};

socket_message s_message;

/**
 * Callback functions of ROS subscribers
 */

void setData();

void sendToServices(autowarePackage::AUTOWAREMCM autoware);

/**
 * Callback functions of ROS subscribers
 */

void storePlannedTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message);

void receiveScenarioTrigger(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message);

void detectCollision(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message);

void validatedDesiredTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message);

void calculatedDesiredTrajectory(std::shared_ptr<WsClient::Connection>, std::shared_ptr<WsClient::InMessage> in_message);

/**
 * Class that connects to AUTOWARE via serial port and offers its data to other modules via ZMQ.
 */
class AutowareService {
public:
	AutowareService(AutowareConfig &config, int argc, char* argv[]);
	~AutowareService();

	// void setData();

	void receiveFromAutoware();

	void testSender(int msgType);

	void receiveFromMcService();

	void sendBackToAutoware(socket_message msg);

	// void sendToServices(autowarePackage::AUTOWAREMCM autoware);

	void loadOpt(int argc, char* argv[]);

private:
	AutowareConfig mConfig;
	GlobalConfig mGlobalConfig;

	CommunicationSender* mSender;
	CommunicationReceiver* mReceiverFromMcService;
	LoggingUtility* mLogger;

	boost::thread* mThreadReceive;
	boost::thread* mThreadReceiveFromMcService;

	boost::thread* mThreadTestSender;

	double speed;
	double longitude;
	double latitude;
	double generationUnixTime;

	std::ofstream delay_output_file;

	std::random_device rnd;     // 非決定的な乱数生成器を生成

	int sock_fd;
	int flag;
	std::string host_addr;
};

/** @} */ //end group

#endif
