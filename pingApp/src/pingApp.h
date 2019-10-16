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
 * @addtogroup httpServer
 * @{
 */

#include <boost/thread.hpp>
#include <common/config/config.h>
#include <common/utility/CommunicationReceiver.h>
#include <common/utility/CommunicationSender.h>
#include <common/utility/Constants.h>
#include <common/buffers/data.pb.h>
#include <common/buffers/ItsPduHeader.pb.h>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <mutex>
#include <common/asn1/CAM.h>
#include <common/messages/MessageUtils.h>
#include <ctime>
#include <iostream>


/** A Server that connects to LDMs via ZMQ and exposes it's Data via http.
 * Uses the Crow Framework (<a href="https://github.com/ipkn/crow"> git link</a>) for the http service.
 * The received data/messages from LDM is serialized as Protobuffer and converted to JSON with the library pbjson.
 * In addition to the serialized messages, the JSON string also includes the type and the number of the messages.
 */

struct PingConfig{
    bool mIsRSU;
    void loadConfig(ptree& pt) {
        mIsRSU = pt.get("cam.isRSU", false);
	}
};

class PingApp {
public:

	PingApp();
	virtual ~PingApp();

    /**
     sent to dcc 
    **/
    void send();

    /**
     receive from dcc 
     **/
    void receive();

private:
    GlobalConfig mGlobalConfig;

	CommunicationSender* mSenderToDcc;
    CommunicationReceiver* mReceiverFromDcc;

    PingConfig mConfig;

    MessageUtils* mMsgUtils;
	LoggingUtility* mLogger;
};
/** @}*/ //end group
#endif /* PINGAPP_H_ */
