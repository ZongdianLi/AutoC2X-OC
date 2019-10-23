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


#ifndef PINGAPP_H_
#define PINGAPP_H_

/**
 * @addtogroup ldm
 * @{
 */

#include <boost/thread.hpp>
#include <common/utility/CommunicationReceiver.h>
#include <common/utility/CommunicationServer.h>
#include <common/utility/Constants.h>

#include <common/utility/LoggingUtility.h>
#include <sqlite3.h>
#include <common/buffers/cam.pb.h>
#include <common/buffers/denm.pb.h>
#include <common/buffers/gps.pb.h>
#include <common/buffers/obd2.pb.h>
#include <common/buffers/dccInfo.pb.h>
#include <common/buffers/camInfo.pb.h>
#include <common/buffers/ldmData.pb.h>
#include <google/protobuf/text_format.h>
#include <string>
#include <ctime>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <mutex>

/**
 * The Local Dynamic Map (LDM) is responsible for maintaining data that is part of ITS.
 *
 * The data is written into a SQLite database. In addition, the latest data is cached in order to quickly answer requests.
 * (Apparently, sql queries are too slow to answer requests with a high frequency.)
 *
 * @nonStandard deletion of old data. for debugging add "deleted" flag and automatically "delete" entries that are eg. too old, too far away; only return non-delted entries
 *
 * @nonStandard no security checks for requests
 *
 * @nonStandard no registration of dataProviders, dataReceivers
 *
 * @nonStandard no way of general data requests, only latest data available
 */
class pingApp {
public:
	pingApp();
	~pingApp();
	void init();



	/**	Receives CAM.
	 *  Receives CAM from the local or a remote machine, caches the data and inserts it into the database.
	 */
	void receiveFromCa();

	/**	Receives and answers requests.
	 *  Receives requests for querying data from a specified table, calls the corresponding select and finally replies to the request.
	 */
	void receiveRequest();

private:
	CommunicationReceiver* mReceiverFromCa;
	// CommunicationServer* mServer;

	boost::thread* mThreadReceiveFromCa;
	// boost::thread* mThreadServer;


	LoggingUtility* mLogger;

	/**
	 * Cache for storing the latest CAM for each stationId
	 */
	std::map<std::string,camPackage::CAM> camCache;
};

/** @} */ //end group
#endif