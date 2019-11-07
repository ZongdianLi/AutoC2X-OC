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
#include <common/utility/LoggingUtility.h>
#include <common/utility/Constants.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/asio.hpp>
#include <string>
#include <common/config/config.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <zmq.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>


/** Struct that holds the configuration for AutowareService.
 * The configuration is defined in <a href="../../autoware/config/config.xml">autoware/config/config.xml</a>.
 */
struct AutowareConfig {

	bool isReceiver;

	void loadConfig() {
		
		ptree pt = load_config_tree();
		
        isReceiver = pt.get("autoware.isReceiver", true);
		
	}
};

struct message {
	int speed;
};

/**
 * Class that connects to AUTOWARE via serial port and offers its data to other modules via ZMQ.
 */
class AutowareService {
public:
	AutowareService(AutowareConfig &config);
	~AutowareService();

	void messageReceive(message message);

	void messageSend(message message);

private:
	AutowareConfig mConfig;
	GlobalConfig mGlobalConfig;

	CommunicationSender* mSender;
	LoggingUtility* mLogger;


};

/** @} */ //end group

#endif
