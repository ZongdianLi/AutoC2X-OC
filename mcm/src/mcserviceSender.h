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


#ifndef MCSERVICE_H_
#define MCSERVICE_H_

/**
 * @addtogroup mcm
 * @{
 */

#include <boost/thread.hpp>
#include <common/config/config.h>
#include <common/utility/CommunicationReceiver.h>
#include <common/utility/CommunicationSender.h>
#include <common/utility/Constants.h>
#include <common/buffers/data.pb.h>
#include <common/buffers/mcm.pb.h>
#include <common/buffers/gps.pb.h>
#include <common/buffers/obd2.pb.h>
#include <common/buffers/pingApp.pb.h>
#include <common/buffers/mcmInfo.pb.h>
#include <common/buffers/autoware.pb.h>
#include <common/buffers/ManeuverCoordination.pb.h>
#include <common/buffers/ItsPduHeader.pb.h>
#include <common/buffers/autowareMcm.pb.h>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <mutex>
#include <common/asn1/MCM.h>
#include <common/messages/MessageUtils.h>
// #include <common/buffers/autoware.pb.h>
#include <fstream>
#include <chrono>
#include <ctime>


/** Struct that hold the configuration for McService.
 * The configuration is defined in /etc/config/openc2x_mcm</a>
 */
struct McServiceConfig {
	bool mGenerateMsgs;
	int mExpirationTime;
	int mMaxGpsAge;
	int mMaxObd2Age;
	double mThresholdRadiusForHeading;
	bool mIsRSU;

	void loadConfig(ptree& pt) {
		
		
		
		mGenerateMsgs = pt.get("mcm.generateMsgs", true);
		mExpirationTime = pt.get("mcm.expirationTime", 1);
		mMaxGpsAge = pt.get("mcm.maxGpsAge", 10);
		mMaxObd2Age = pt.get("mcm.maxObd2Age", 10);
		mThresholdRadiusForHeading = pt.get("mcm.thresholdRadiusForHeading", 0.3);
		mIsRSU = pt.get("mcm.isRSU", false);

		
	}
};

/**
* message type (ex. IntentionRequest, IntentionReply, ...)
*/
enum Type {
	IntentionRequest = 0,
	IntentionReply = 1,
	Prescription = 2,
	Acceptance = 3,
	Heartbeat = 4,
	Ack = 5,
	Fin = 6
};

/**
* state (ex. Waiting, Advertising, ...)
*/
enum State {
	Waiting = 0,
	Advertising = 2,
	Negotiating = 3,
	Activating = 4
};

/**
 * Class that handles the receiving, creating and sending of CA Messages.
 *
 * @nonStandard Outdated MCMs queued in ath9k hardware can not be flushed. Ath9k driver developer, Adrian Chadd says:
 * <div>
 * "Once they're in the hardware queue then they're there. You'd have to
 * stop the TX queue (which if it fails, means you have to reset the
 * chip) before you can fiddle with the TX queue. But with TX FIFO chips
 * (AR9380 and later), you're pushing TX entries into the FIFO, and you
 * can't remove them. AND, the pre-FIFO chips (AR928x and earlier) the
 * MAC doesn't always re-read TXDP when you start TX - it only is
 * guaranteed to read it once, after a reset, and after that you
 * shouldn't overwrite it or it occasionally ignores you."
 * </div>
 */
class McService {
public:
	McService(McServiceConfig &config, ptree& configTree);
	~McService();

	/** Sends a new MCM to LDM and DCC.	 */
	void send(bool isAutoware = false, Type type = IntentionRequest);

	// /** Calculates the heading towards North based on the two specified coordinates.
	//  *
	//  * @param lat1 Latitude of coordinate 1.
	//  * @param lon1 Longitude of coordinate 1.
	//  * @param lat2 Latitude of coordinate 2.
	//  * @param lon2 Longitude of coordinate 2.
	//  * @return The heading in degrees.
	//  */
	// double getHeading(double lat1, double lon1, double lat2, double lon2);

	// /** Calculates the distance between the two specified coordinates
	//  *
	//  * @param lat1 Latitude of coordinate 1.
	//  * @param lon1 Longitude of coordinate 1.
	//  * @param lat2 Latitude of coordinate 2.
	//  * @param lon2 Longitude of coordinate 2.
	//  * @return The distance in meters.
	//  */
	// double getDistance(double lat1, double lon1, double lat2, double lon2);

private:
	/** Receives incoming MCMs from DCC and forwards them to LDM.
	 * Receives serialized DATA packages from DCC, deserializes it, and forwards the contained serialized MCM to LDM.
	 */
	void receive();

	/** Sends information about why a MCM was triggered to LDM.
	 * Sends serialized mcmInfo to LDM, including the current time and the specified reason for triggering.
	 * @param triggerReason Difference in time, heading, position, or speed.
	 * @param delta Specifies how big the difference in time, heading, position, or speed is.
	 */
	void sendMcmInfo(std::string triggerReason, double delta);

	/** Periodically checks the rules for triggering a new MCM.
	 * Checks every 100ms whether the difference in time, heading, position, or speed requires the triggering of a new MCM. Reasons for triggering are:
	 * The time difference since the last triggered MCM is at least one second.
	 * The heading since the last MCM changed by more than four degrees.
	 * The position since the last MCM changed by more than five meters.
	 * The speed since the last MCM changed by more than 1m/s.
	 * @param ec Boost error code
	 */
	void alarm(const boost::system::error_code &ec, Type type);

	/** Triggers sending of MCM.
	 *
	 */
	void trigger(Type type, int interval);

	/** Generates a new unaligned PER compliant MCM.
	 * The new MCM includes the MAC address as stationId, an increasing but not unique ID, a current time stamp, and the latest GPS and OBD2 data if it is not too old (as configured).
	 * @return The newly generated MCM.
	 */
	/*std::vector<uint8_t>*/MCM_t* generateMcm(bool isAutoware = false, Type type = IntentionRequest);

	/** Converts ASN1 MCM structure into MCM protocol buffer.
	 * @return The newly generated MCM protocol buffer.
	 */
	mcmPackage::MCM convertAsn1toProtoBuf(MCM_t* mcm);

	// /** Receives new GPS data from the GPS module.
	//  *
	//  */
	// void receiveGpsData();

	// /** Receives new OBD2 data from the OBD2 module.
	//  *
	//  */
	// void receiveObd2Data();

	/** Receives new AUTOAWRE data from the AUTOWARE module.
	 *
	 */
	void receiveAutowareData();
	// void receiveReflectedData();

	// /** Checks if heading has changed more than 4 degrees.
	//  * @return True if MCM needs to be triggered, false otherwise
	//  */
	// bool isHeadingChanged();

	// /** Checks if position has changed more than 5 metres.
	//  * @return True if MCM needs to be triggered, false otherwise
	//  */
	// bool isPositionChanged();

	// /** Checks if speed has changed more than 1 m/sec.
	//  * @return True if MCM needs to be triggered, false otherwise
	//  */
	// bool isSpeedChanged();

	// /** Checks if speed has changed more than 1 m/sec.
	//  * @return True if MCM needs to be triggered, false otherwise
	//  */
	// bool isAutowareSpeedChanged();

	// /** Checks if time more than 1 second has past since last MCM.
	//  * @return True if MCM needs to be triggered, false otherwise
	//  */
	bool isTimeToTriggerMCM();

	/** Schedules next triggering checks for new MCM.
	 *
	 */
	void scheduleNextAlarm(Type type, int interval);

	// /** Checks if the last received GPS data is still valid.
	//  * @return True if GPS data is valid, false otherwise.
	//  */
	// bool isGPSdataValid();
	

	GlobalConfig mGlobalConfig;
	McServiceConfig mConfig;

	CommunicationSender* mSenderToDcc;
	CommunicationSender* mSenderToLdm;
	CommunicationSender* mSenderToAutoware; //反射してきたpingパケットをautowareモジュールに送るために必要

	CommunicationReceiver* mReceiverFromDcc;
	// CommunicationReceiver* mReceiverGps;
	// CommunicationReceiver* mReceiverObd2;
	CommunicationReceiver* mReceiverAutoware;
	CommunicationReceiver* mReceiverPingApp;

	boost::thread* mThreadReceive;
	// boost::thread* mThreadGpsDataReceive;
	// boost::thread* mThreadObd2DataReceive;
	boost::thread* mThreadAutowareDataReceive;
	boost::thread* mThreadPingAppDataReceive;

	MessageUtils* mMsgUtils;
	LoggingUtility* mLogger;

	boost::asio::io_service mIoService;
	boost::asio::deadline_timer* mTimer;

	long mIdCounter;
	double mMcmTriggerInterval;

	gpsPackage::GPS mLatestGps;
	/**
	 * True iff a GPS was received and it is not too old.
	 */
	bool mGpsValid;		//true if GPS was received and it's not too old
	std::mutex mMutexLatestGps;

	obd2Package::OBD2 mLatestObd2;
	/**
	 * True iff a OBD2 was received and it is not too old.
	 */
	bool mObd2Valid;
	std::mutex mMutexLatestObd2;

	autowarePackage::AUTOWAREMCM mLatestAutoware;
	bool mAutowareValid;
	std::mutex mMutexLatestAutoware;
	pingAppPackage::PINGAPP mLatestPingApp;
	bool mPingAppValid;
	std::mutex mMutexLatestPingApp;

	struct LastSentMcmInfo {
		bool hasGPS = false;
		gpsPackage::GPS lastGps;
		bool hasOBD2 = false;
		obd2Package::OBD2 lastObd2;
		bool hasAUTOWARE = false;
		autowarePackage::AUTOWARE lastAutoware;
		double lastHeading;
		int64_t timestamp = 0;
	};
	LastSentMcmInfo mLastSentMcmInfo;

	std::ofstream atoc_delay_output_file;

	//std::vector<autowarePackage::AUTOWARE> waiting_data;
	// std::list<autowarePackage::AUTOWAREMCM> waiting_data;
	autowarePackage::AUTOWAREMCM waiting_data;

	enum Type type;
	enum State state = Waiting;

	bool ack = true;
	bool fin = true;
};

/** @} */ //end group
#endif
