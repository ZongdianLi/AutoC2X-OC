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
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <common/utility/CommunicationSender.h>
#include <common/utility/LoggingUtility.h>
#include <common/utility/Constants.h>
#include <common/buffers/autoware.pb.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/asio.hpp>
#include <string>
#include <common/config/config.h>
#include "projects.h"
#include <chrono>
#include <ctime>
#include <fstream>


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

/**
 * Class that connects to AUTOWARE via serial port and offers its data to other modules via ZMQ.
 */
class AutowareService {
public:
	// AutowareService(AutowareConfig &config, std::string globalConfig, std::string loggingConf, std::string statisticConf);
	AutowareService(AutowareConfig &config);
	~AutowareService();
	static void init(ros::NodeHandle tmp);

	/** Reads the actual vehicle data from AUTOWARE and distributes it via ZMQ.
	 *  Periodically reads speed and rpm data from AUTOWARE if simulating data is turned off,
	 *	writes it into a protocol buffer and sends it to services such as CaService and DenService.
	 *	@param ec Boost error code.
	 *	@param serial SerialPort that handles the actual connection to AUTOWARE via serial port.
	 */
	static void receiveData(const boost::system::error_code &ec, SerialPort* serial);

	/** Simulates vehicle speed.
	 *  Simulates vehicle speed using a random distribution.
	 */
	static void simulateSpeed();

	/** Simulates vehicle data and distributes it via ZMQ.
	 *  Periodically simulates speed data if simulating data is turned on,
	 *	writes it into a protocol buffer and sends it to services such as CaService and DenService.
	 *	@param ec Boost error code.
	 */
	static void simulateData();

	/** Distributes AUTOWARE via ZMQ.
	 *  Sends AUTOWARE via ZMQ to services such as CaService and DenService and logs the data.
	 *  @param autoware The AUTOWARE to be sent to services.
	 */
	static void sendToServices(autowarePackage::AUTOWARE autoware);

	static void callback(const geometry_msgs::PoseStamped msg);

	static double calcSpeed();

	static std::string paramOrganize(std::string param);

	static void timeCalc();

private:
	static AutowareConfig mConfig;
	static GlobalConfig mGlobalConfig;

	static CommunicationSender* mSender;
	static LoggingUtility* mLogger;

	//for simulation only
	static std::default_random_engine mRandNumberGen;
	static std::bernoulli_distribution mBernoulli;
	static std::uniform_real_distribution<double> mUniform;

	static boost::asio::io_service mIoService;
	static boost::asio::deadline_timer* mTimer;

	static ros::NodeHandle *n;
	static double speed;
	static double longitude;
	static double latitude;
	static double generationUnixTime;
	static geometry_msgs::PoseStamped nowPose;
	static geometry_msgs::PoseStamped prevPose;

	static PJ *p_proj;
	static std::ofstream delay_output_file;

};

/** @} */ //end group

#endif