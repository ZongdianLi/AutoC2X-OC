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

#include "SerialPort.h"
#include <common/utility/CommunicationSender.h>
#include <common/utility/CommunicationReceiver.h>
#include <common/utility/CommunicationClient.h>
#include <common/utility/LoggingUtility.h>
#include <common/utility/Constants.h>
#include <common/buffers/autoware.pb.h>
#include <common/buffers/cam.pb.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <string>
#include <common/config/config.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <random>

#include <common/buffers/cam.pb.h>
#include <common/buffers/denm.pb.h>
#include <common/buffers/gps.pb.h>
#include <common/buffers/obd2.pb.h>
#include <common/buffers/dccInfo.pb.h>
#include <common/buffers/camInfo.pb.h>
#include <common/buffers/ldmData.pb.h>

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>


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


struct socket_message{
	long timestamp;
	std::vector<int> speed;
	std::vector<int> latitude;
	std::vector<int> longitude;
	std::vector<int> time;
private:
	// friend class boost::serialization::access;
	friend class cereal::access;
	template<class Archive>
		void serialize( Archive& ar, std::uint32_t version){
			ar & timestamp;
			ar & speed;
			ar & latitude;
			ar & longitude;
			ar & time;
		}
};

/**
 * Class that connects to AUTOWARE via serial port and offers its data to other modules via ZMQ.
 */
class AutowareService {
public:
	// AutowareService(AutowareConfig &config, std::string globalConfig, std::string loggingConf, std::string statisticConf);
	AutowareService(AutowareConfig &config);
	~AutowareService();
	void init();

	void receiveData(const boost::system::error_code &ec, SerialPort* serial);
	
	void simulateData();
	void receiveFromCa();

	double calcSpeed();

	void timeCalc();

	void sendToAutoware(long timestamp);

	void testSender();

	void receiveFromAutoware();




private:
	AutowareConfig mConfig;
	GlobalConfig mGlobalConfig;

	CommunicationSender* mSender;
	LoggingUtility* mLogger;

	CommunicationReceiver* mReceiverFromCa;

	boost::thread* mThreadReceiveFromCa;

	//for simulation only
	std::default_random_engine mRandNumberGen;
	std::bernoulli_distribution mBernoulli;
	std::uniform_real_distribution<double> mUniform;

	boost::asio::io_service mIoService;
	boost::asio::deadline_timer* mTimer;

	boost::thread* mThreadReceive;
	boost::thread* mThreadReceiveFromAutoware;

	double speed;
	double longitude;
	double latitude;
	double generationUnixTime;

	int sockfd;
	int flag;

	std::ofstream delay_output_file;

	socket_message s_message;	
	socket_message tmp_message;	

	std::random_device rnd;

	CommunicationClient* mClientCam;

	long newestLdmKey;

};

/** @} */ //end group

#endif
