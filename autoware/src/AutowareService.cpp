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

#include "AutowareService.h"
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <common/utility/Utils.h>
#include <math.h>

using namespace std;

INITIALIZE_EASYLOGGINGPP


AutowareConfig AutowareService::mConfig;
GlobalConfig AutowareService::mGlobalConfig;
CommunicationSender* AutowareService::mSender;
LoggingUtility* AutowareService::mLogger;

	//for simulation only
std::default_random_engine AutowareService::mRandNumberGen;
std::bernoulli_distribution AutowareService::mBernoulli;
std::uniform_real_distribution<double> AutowareService::mUniform;

boost::asio::io_service AutowareService::mIoService;
boost::asio::deadline_timer* AutowareService::mTimer;

ros::NodeHandle *AutowareService::n;
double AutowareService::speed;
double AutowareService::latitude;
double AutowareService::longitude;
geometry_msgs::PoseStamped AutowareService::nowPose;
geometry_msgs::PoseStamped AutowareService::prevPose;

PJ *AutowareService::p_proj;


AutowareService::AutowareService(AutowareConfig &config) {
	try {
		mGlobalConfig.loadConfig(AUTOWARE_CONFIG_NAME);
	}
	catch (std::exception &e) {
		cerr << "Error while loading /etc/config/openc2x_common: " << e.what() << endl;
	}
	mConfig = config;
	ptree pt = load_config_tree();
	mLogger = new LoggingUtility(AUTOWARE_CONFIG_NAME, AUTOWARE_MODULE_NAME, mGlobalConfig.mLogBasePath, mGlobalConfig.mExpName, mGlobalConfig.mExpNo, pt);

	mSender = new CommunicationSender("25000", *mLogger);
	// mLogger = new LoggingUtility("AutowareService", mGlobalConfig.mExpNo, loggingConf, statisticConf);
	mLogger->logStats("speed (m/sec)");
	
	//for simulation only
	mRandNumberGen = default_random_engine(0);
	mBernoulli = bernoulli_distribution(0);
	mUniform = uniform_real_distribution<double>(-0.01, 0.01);

}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	mTimer->cancel();
	delete mTimer;
}

//reads the actual vehicle data from Autoware
void AutowareService::receiveData(const boost::system::error_code &ec, SerialPort* serial) {
	// double speed = serial->readSpeed();
	// int rpm = serial->readRpm();
	// cout << to_string(speed) << " " << to_string(rpm) << endl;
	// if (speed != -1) {		//valid speed
	// 	//write current data to protocol buffer
	// 	autowarePackage::AUTOWARE autoware;
	// 	autoware.set_time(Utils::currentTime());
	// 	autoware.set_speed(speed * 100); // standard expects speed in 0.01 m/s
	// 	if (rpm != -1) {
	// 		autoware.set_rpm(rpm);
	// 	}
	// 	sendToServices(autoware);
	// }
	// mTimer->expires_from_now(boost::posix_time::millisec(mConfig.mFrequency));
	// mTimer->async_wait(boost::bind(&AutowareService::receiveData, this, boost::asio::placeholders::error, serial));
}


//simulates realistic vehicle speed
void AutowareService::simulateSpeed() {
	double pCurr = mBernoulli.p();
	double pNew = pCurr + mUniform(mRandNumberGen);
	pNew = min(0.15, max(0.0, pNew));		//pNew always between 0 and 0.15

	mBernoulli = bernoulli_distribution(pNew);

	double sum = 0;

	for (int i=0; i<1000; i++) {
		double r = mBernoulli(mRandNumberGen);
		sum += min(1.0, max(0.0, r)) * 100;	//*100 to convert to 0-15 m/s
	}
	cout << to_string(sum) << endl;
	speed = sum / 1000.0;
	// return sum / 1000.0;					//avg to avoid rapid/drastic changes in speed
}

//simulates Autoware data, logs and sends it
void AutowareService::simulateData() {
	autowarePackage::AUTOWARE autoware;

	//write current speed to protocol buffer
	autoware.set_speed(speed * 100); // standard expects speed in 0.01 m/s
	autoware.set_time(Utils::currentTime());

	autoware.set_longitude(longitude);
	autoware.set_latitude(latitude);
	sendToServices(autoware);

	// mTimer->expires_from_now(boost::posix_time::millisec(mConfig.mFrequency));
	// mTimer->async_wait(boost::bind(&AutowareService::simulateData, this, boost::asio::placeholders::error));
}

//logs and sends Autoware
void AutowareService::sendToServices(autowarePackage::AUTOWARE autoware) {
	//send buffer to services
	string serializedAutoware;
	autoware.SerializeToString(&serializedAutoware);
	mSender->sendData("AUTOWARE", serializedAutoware);
	mLogger->logStats(to_string(autoware.speed()) + " (" + to_string(autoware.speed()/100*3.6) + "km/h)"); // In csv, we log speed in m/sec
}

std::string AutowareService::paramOrganize(std::string param){
	char **prm;
	std::string params;
  params=param+" no_defs";
  istringstream si(params);

  int n_pair=1;         // add one for last token "no_defs"
  for ( unsigned long i=0; i < params.size(); i++ )
    if ( params[i] == '=' ) n_pair++;

  prm = new char *[n_pair];
  for ( int i=0; i < n_pair; i++ ) {
    prm[i] = new char[256];
    si >> prm[i];
  }

  p_proj=pj_init(n_pair, prm);
  

  for ( int i=n_pair-1; i >=0; i-- ) {
    delete [] prm[i];
  }
  delete [] prm;

  if ( !p_proj ) {
    cerr << "Failed to initialize the PROJ library\n";
    exit(1);
  }

	return params;
 
}

void AutowareService::init(ros::NodeHandle tmp) {
	paramOrganize("proj=tmerc lat_0=40 lon_0=140.8333333333333 k=0.9999 x_0=0 y_0=0 ellps=GRS80 units=m");
	// paramOrganize("proj=poly ellps=sphere lon_0=100 lat_0=45");
	n = &tmp;
	// ros::Subscriber sub = n->subscribe("ndt_pose", 1024, callback);
	if (!mConfig.mSimulateData) {	//use real Autoware data
		// SerialPort* serial = new SerialPort();
		// if (serial->connect(mConfig.mDevice) != -1) {
		// 	mLogger->logInfo("Connected to serial port successfully");

		// 	serial->init();

		// 	mTimer = new boost::asio::deadline_timer(mIoService, boost::posix_time::millisec(mConfig.mFrequency));
		// 	mTimer->async_wait(boost::bind(&AutowareService::receiveData, this, boost::asio::placeholders::error, serial));
		// 	mIoService.run();

		// 	serial->disconnect();
		// }
		// else {
		// 	mLogger->logError("Cannot open serial port -> plug in Autoware and run with sudo");
		// }
	}
	else {				//use simulated Autoware data
		// mTimer = new boost::asio::deadline_timer(mIoService, boost::posix_time::millisec(mConfig.mFrequency));
		// mTimer->async_wait(boost::bind(&AutowareService::simulateData, this, boost::asio::placeholders::error));
		// mIoService.run();
	}
	ros::spin();

}

double AutowareService::calcSpeed(){
	double prevTime = prevPose.header.stamp.sec + prevPose.header.stamp.nsec/1000000000.0;
	double nowTime = nowPose.header.stamp.sec + nowPose.header.stamp.nsec/1000000000.0;
	double timedelta = nowTime - prevTime;

	double d_x = pow(prevPose.pose.position.x - nowPose.pose.position.x, 2);
	double d_y = pow(prevPose.pose.position.y - nowPose.pose.position.y, 2);

	// double DEG_TO_RAD = M_PI / 180;
	projUV xy;
	xy.u = nowPose.pose.position.x;
	xy.v = nowPose.pose.position.y;
	projUV result = pj_inv(xy, p_proj);
	result.u /= DEG_TO_RAD;
	result.v /= DEG_TO_RAD;
	printf("lng: %f, lat: %f", result.u, result.v); 

	longitude = result.u;
	latitude = result.v;

	return sqrt(d_x + d_y) / timedelta;
}

void callback(const autoware_msgs::DetectedObjectArray msg){
// void AutowareService::callback(const geometry_msgs::PoseStamped msg){
	// sensor_msgs::PointCloud out_pointcloud;
	// sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
	// printf("%d\n", msg.header.seq);
	// for(int i = 0 ; i < out_pointcloud.points.size(); ++i){
	// geometry_msgs::Point32 point;
	//  x = -6372.1474609375, y = -31747.03125,
	//Dooo something here
	// point.z = out_pointcloud.points[i].z;
	// printf("%f\n", point.z);
	// }
	// printf("%d\n", msg.width);
    // printf("%f\n", msg.pose.position.x);

	// prevPose = nowPose;
	// nowPose = msg;

	const autoware_msgs::DetectedObjectArray message = msg;
	// speed = calcSpeed();
	// printf("%f\n", speed);
	// simulateData();
}



std::vector<geometry_msgs::Point32> createLine(){
	std::vector<geometry_msgs::Point32> result;
	int N = 50;
	for(int x = 0; x < N; x++){
		for(int y = 0; y < N; y++){
			for(int z = 0; z < N; z++){
				geometry_msgs::Point32 p;
				p.x = x*0.1 + 10;
				p.y = y*0.1;
				p.z = z*0.1;
				result.push_back(p);
			}
		}
	}
	return result;
}

std::vector<geometry_msgs::Point32> createConvexHull(){
	std::vector<geometry_msgs::Point32> result;

	geometry_msgs::Point32 a1, a2, a3, a4;

	a1.x = 10;
	a1.y = 0;
	a1.z = 0;
	
	a2.x = 15;
	a2.y = 0;
	a2.z = 0;

	a3.x = 15;
	a3.y = 5;
	a3.z = 0;

	a4.x = 10;
	a4.y = 5;
	a4.z = 0;

	result.push_back(a1);
	result.push_back(a2);
	result.push_back(a3);
	result.push_back(a4);
	result.push_back(a1);

	return result;
}

sensor_msgs::ChannelFloat32 createChannel(std::string name){
	sensor_msgs::ChannelFloat32 result;
	int N = 50;
	result.name = name;

	std::vector<float> values;
	for(int i=0; i<N*N*N ; i++){
		values.push_back(0.9);
	}

	result.values = values;
	return result;
}

void createObjectsPublisher(const ros::TimerEvent&){
	ros::NodeHandle n;	
	ros::Publisher pub = n.advertise<autoware_msgs::DetectedObjectArray>("detection/lidar_detector/objects", 1000);
	// ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("points_cluster", 1000);

	autoware_msgs::DetectedObjectArray msg;
	std::vector<autoware_msgs::DetectedObject> objects;

	ros::Time nowTime = ros::Time::now();
	nowTime.sec = nowTime.sec + 1;

	msg.header.stamp = nowTime;
	msg.header.frame_id = "velodyne";

	autoware_msgs::DetectedObject object;
	object.header.stamp = nowTime;
	object.header.frame_id = "velodyne";
	object.label = "unknown";
	object.valid = 1;
	object.score = 1;
	object.space_frame = "velodyne";
	object.pose.position.x = 5;
	object.pose.position.y = -10;
	object.pose.orientation.w = 1;
	object.dimensions.x = 16.3;
	object.dimensions.y = 4.06;
	object.dimensions.z = 2.34;


	std::vector<geometry_msgs::Point32> points = createLine();
	std::vector<sensor_msgs::ChannelFloat32> channels;
	channels.push_back( createChannel("rgb") );
	sensor_msgs::PointCloud input_msg;
	sensor_msgs::PointCloud2 output_msg;
	input_msg.header.stamp = nowTime;
	input_msg.header.frame_id = "velodyne";
	input_msg.points = points;
	input_msg.channels = channels;
	sensor_msgs::convertPointCloudToPointCloud2(input_msg, output_msg);
	object.pointcloud = output_msg;

	geometry_msgs::PolygonStamped convex_hull_msg;
	convex_hull_msg.header.stamp = nowTime;
	convex_hull_msg.header.frame_id = "velodyne";
	geometry_msgs::Polygon polygon;
	polygon.points = createConvexHull();
	convex_hull_msg.polygon = polygon;
	object.convex_hull = convex_hull_msg;

	objects.push_back(object);
	msg.objects = objects;
	pub.publish(msg);

	printf("published %d.%d\n",nowTime.sec, nowTime.nsec);
}

void createPointsPublisher(const ros::TimerEvent&){
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("points_cluster", 1000);

	ros::Rate loop_rate(0.1);

	std::vector<geometry_msgs::Point32> points = createLine();
	std::vector<sensor_msgs::ChannelFloat32> channels;
	channels.push_back( createChannel("rgb") );

	// while (ros::ok()){
	ros::Time nowTime = ros::Time::now();
	nowTime.sec = nowTime.sec + 1;
	
	sensor_msgs::PointCloud input_msg;
	sensor_msgs::PointCloud2 output_msg;

	input_msg.header.stamp = nowTime;
	input_msg.header.frame_id = "velodyne";
	input_msg.points = points;
	input_msg.channels = channels;

	sensor_msgs::convertPointCloudToPointCloud2(input_msg, output_msg);
	pub.publish(output_msg);
	printf("published:%d\n", output_msg.header.seq);

		// ros::spinOnce();
		// loop_rate.sleep();

	// }
	

}



int main(int argc,  char* argv[]) {


	AutowareConfig config;
	try {
		config.loadConfig();
	}
	catch (std::exception &e) {
		cerr << "Error while loading config.xml: " << e.what() << endl << flush;
		return EXIT_FAILURE;
	}
	AutowareService autoware(config);
	
	ros::init(argc, argv, "listener");
	ros::NodeHandle n, n2;

    // ros::Subscriber sub = n.subscribe("ndt_pose", 1024, callback);
    // ros::Subscriber sub = n.subscribe("points_raw", 1024, callback);
	// ros::Subscriber sub = n.subscribe("points_cluster", 1024, callback);
	// ros::Subscriber sub = n.subscribe("detection/lidar_detector/objects", 1024, callback);
	ros::Timer timer = n.createTimer(ros::Duration(0.1), createObjectsPublisher); //こっちでより下流のトピックに流し込む
	// ros::Timer timer = n.createTimer(ros::Duration(0.1), createPointsPublisher); //こっちはグレーの四角が挿入される

	// autoware.init(n);
	ros::spin();
	return 0;
}

/* points_raw message
 {header = {seq = 5339, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494474168, nsec = 386117000}, <No data fields>}, frame_id = "velodyne"}, 
  height = 1,
  width = 25430, 
  fields = std::vector of length 5, capacity 5 = {{name = "x", offset = 0, datatype = 7 '\a', count = 1}, {name = "y", offset = 4, 
      datatype = 7 '\a', count = 1}, {name = "z", offset = 8, datatype = 7 '\a', count = 1}, {name = "intensity", offset = 16, datatype = 7 '\a', count = 1}, {
      name = "ring", offset = 20, datatype = 4 '\004', count = 1}}, 
  is_bigendian = 0 '\000', 
  point_step = 32, 
  row_step = 813760, 
  data = std::vector of length 813760, capacity 813760 = {229 '\345', 152 '\230', 180 '\264', 64 '@', 138 '\212', 197 '\305', 81 'Q', 65 'A', 83 'S', 33 '!', 
    127 '\177', 62 '>', 123 '{', 127 '\177', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 128 '\200', 63 '?', 8 '\b', 0 '\000', 0 '\000', 0 '\000', 165 '\245', 8 '\b', 
    0 '\000', 0 '\000', 1 '\001', 0 '\000', 0 '\000', 0 '\000', 225 '\341', 0 '\000', 183 '\267', 64 '@', 248 '\370', 118 'v', 84 'T', 65 'A', 241 '\361', 
    249 '\371', 65 'A', 63 '?', 123 '{', 127 '\177', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 64 '@', 9 '\t', 0 '\000', 0 '\000', 0 '\000', 165 '\245', 
    8 '\b', 0 '\000', 0 '\000', 1 '\001', 0 '\000', 0 '\000', 0 '\000', 210 '\322', 74 'J', 15 '\017', 64 '@', 217 '\331', 71 'G', 166 '\246', 64 '@', 1 '\001', 
    199 '\307', 140 '\214', 191 '\277', 123 '{', 127 '\177', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 168 '\250', 65 'A', 2 '\002', 0 '\000', 0 '\000', 0 '\000', 
    165 '\245', 8 '\b', 0 '\000', 0 '\000', 1 '\001', 0 '\000', 0 '\000', 0 '\000', 58 ':', 85 'U', 180 '\264', 64 '@', 188 '\274', 41 ')', 81 'Q', 65 'A', 
    22 '\026', 107 'k', 159 '\237', 63 '?', 123 '{', 127 '\177', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 160 '\240', 64 '@', 10 '\n', 0 '\000', 0 '\000', 
    0 '\000', 165 '\245', 8 '\b', 0 '\000', 0 '\000', 1 '\001', 0 '\000', 0 '\000', 0 '\000', 108 'l', 78 'N', 38 '&', 64 '@', 221 '\335', 228 '\344', 192 '\300', 
    64 '@', 178 '\262', 19 '\023', 133 '\205', 191 '\277', 123 '{', 127 '\177', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 64 '@', 3 '\003', 0 '\000', 
    0 '\000', 0 '\000', 165 '\245', 8 '\b', 0 '\000', 0 '\000', 1 '\001', 0 '\000', 0 '\000', 0 '\000', 39 '\'', 118 'v', 202 '\312', 64 '@', 141 '\215', 
    183 '\267', 106 'j', 65 'A', 2 '\002', 22 '\026', 251 '\373', 63 '?', 123 '{', 127 '\177', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 11 '\v', 
    0 '\000', 0 '\000', 0 '\000', 165 '\245', 8 '\b', 0 '\000', 0 '\000', 1 '\001', 0 '\000', 0 '\000', 0 '\000', 86 'V', 209 '\321', 2 '\002', 64 '@', 28 '\034', 
    150 '\226', 151 '\227', 64 '@'...}, 
  is_dense = 1 '\001'}
*/

/*
{header = {seq = 5971, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494474234, nsec = 646776000}, <No data fields>}, frame_id = "velodyne"}, 
  points = std::vector of length 27068, capacity 27068 = {{x = -4.45708418, y = -7.89715528, z = -2.42979336}, {x = -23.9344826, y = -42.3903313, 
      z = 0.849722803}, {x = -5.35591841, y = -9.48586082, z = -2.51495266}, {x = -24.018795, y = -42.5223236, z = 2.55943799}, {x = -6.57362795, y = -11.6330585, 
      z = -2.59729218}, {x = -24.0642624, y = -42.5681152, z = 4.27812672}, {x = -8.29219818, y = -14.6683578, z = -2.66877246}, {x = -24.1633358, 
      y = -42.7259598, z = 6.02692699}, {x = -7.20689917, y = -12.7381487, z = 2.31804609}, {x = -7.83165693, y = -13.8311443, z = 3.08957958}, {x = -23.9783688, 
      y = -42.312664, z = -0.848919988}, {x = -7.64205456, y = -13.4798307, z = 4.15197515}, {x = -4.48271275, y = -7.87819147, z = -2.42875814}, {
      x = -24.1006355, y = -42.3559151, z = 0.850630283}, {x = -5.38707018, y = -9.46372032, z = -2.51405311}, {x = -24.1710339, y = -42.4451675, z = 2.55985665}, 

  channels = std::vector of length 2, capacity 2 = {
	  {name = "intensity", values = std::vector of length 27068, capacity 27068 = {16, 36, 1, 42, 2, 44, 3, 40, 13 
        3, 63, 11, 16, 36, 1, 42, 2, 44, 3, 40, 9, 13, 57, 28, 63, 7, 16, 36, 1, 42, 2, 45, 3, 11, 11, 16, 17, 28, 63, 47, 16, 36, 1, 43, 2, 45, 3, 6, 20, 13, 14, 
        27, 63, 16, 36, 1, 42, 2, 44, 3, 2, 16, 13, 14, 26, 63, 16, 35, 1, 2, 43, 3, 14, 3, 13, 14, 12, 63, 44, 16, 34, 1, 40, 2, 3, 21, 16, 13, 3, 12, 63, 42, 
        16, 1, 40, 2, 41, 3, 9, 16, 13, 10, 16, 41, 16, 31, 1, 2, 40, 3, 15, 16, 21, 40, 16, 30, 1, 37, 2, 38, 3, 5, 24, 10, 63, 39, 16, 29, 1, 2, 37, 3, 10, 12, 
        11, 21, 1, 38, 16, 28, 1, 33, 2, 37, 3, 10, 3, 10, 15, 1, 63, 37, 16, 27, 1, 32, 2, 35, 3, 17, 20, 11, 16, 41, 8, 63, 36, 16, 25, 1, 31, 2, 3, 3, 4, 16, 
        11, 28, 39, 16, 63, 36, 16, 25, 1, 30, 2, 13, 3, 9, 13, 11, 26, 37, 20, 63, 35, 16, 23, 1...}}, 
	  {name = "ring", values = std::vector of length 27068, capacity 27068 = {0, 8, 1, 9, 2, 10, 3, 11, 12, 13, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 6, 14, 7, 15, 0, 8, 1, 9,2, 10, 3, 11, 12, 5, 13, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 13, 14, 7, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 13, 14, 7, 0, 8, 1, 2, 10, 3, 11, 12, 5, 13, 14, 7, 15, 0, 8, 1, 9, 2, 3, 11, 12, 5, 13, 14, 7, 15, 0, 1, 9, 2, 10, 3, 11, 12, 5, 13, 14, 15, 0, 8, 1, 2, 10, 3, 11, 12, 13, 15, 0, 8, 1, 9, 2, 10,3 , 11, 12, 13, 7, 15, 0, 8, 1, 2, 10, 3, 11, 12, 5, 13, 14, 15, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 13, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1, 9, 2, 10, 3, 11, 12, 5, 13, 6, 14, 7, 15, 0, 8, 1...}}
	  }
*/
/*
	/points_cluster
	{
		header = {seq = 75, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {
        sec = 1494473986, nsec = 903388000}, <No data fields>}, frame_id = "velodyne"}, 
    height = 1, 
		width = 6360, 
		fields = std::vector of length 4, capacity 4 = {{name = "x", 
      offset = 0, datatype = 7 '\a', count = 1}, {name = "y", offset = 4, datatype = 7 '\a', 
      count = 1}, {name = "z", offset = 8, datatype = 7 '\a', count = 1}, {name = "rgb", 
      offset = 16, datatype = 7 '\a', count = 1}}, 
		is_bigendian = 0 '\000', 
		point_step = 32, 
    row_step = 203520, 
		data = std::vector of length 203520, capacity 203520 = {144 '\220', 60 '<', 
    145 '\221', 192 '\300', 9 '\t', 13 '\r', 251 '\373', 192 '\300', 111 'o', 235 '\353', 
    5 '\005', 192 '\300', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 
    151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 
    240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 69 'E', 68 'D', 157 '\235', 
    192 '\300', 64 '@', 208 '\320', 7 '\a', 193 '\301', 71 'G', 10 '\n', 244 '\364', 191 '\277', 
    0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 
    0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 
    127 '\177', 0 '\000', 0 '\000', 39 '\'', 47 '/', 169 '\251', 192 '\300', 232 '\350', 
    11 '\v', 18 '\022', 193 '\301', 41 ')', 218 '\332', 213 '\325', 191 '\277', 0 '\000', 
    0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 
    0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 
    0 '\000', 0 '\000', 254 '\376', 96 '`', 204 '\314', 192 '\300', 21 '\025', 55 '7', 48 '0', 
    193 '\301', 166 '\246', 146 '\222', 142 '\216', 191 '\277', 0 '\000', 0 '\000', 128 '\200', 
    63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 
    64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 
    68 'D', 18 '\022', 204 '\314', 192 '\300', 255 '\377', 206 '\316', 47 '/', 193 '\301', 
---Type <return> to continue, or q <return> to quit---
    15 '\017', 115 's', 42 '*', 191 '\277', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 
    81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 
    255 '\377', 240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 87 'W', 139 '\213', 
    236 '\354', 192 '\300', 202 '\312', 179 '\263', 75 'K', 193 '\301', 121 'y', 145 '\221', 
    131 '\203', 190 '\276', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 
    151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 
    240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 182 '\266', 127 '\177', 158 '\236', 
    192 '\300', 71 'G', 200 '\310', 7 '\a', 193 '\301'...}, is_dense = 1 '\001'}

*/

// detected objects array
/*
{header = {seq = 3385, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494473977, nsec = 362651000}, <No data fields>}, 
    frame_id = "velodyne"}, 
objects = std::vector of length 29, capacity 29 = 
				{{header = {seq = 4405, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {
            sec = 1494473977, nsec = 362651000}, <No data fields>}, frame_id = "velodyne"},
					id = 0, 
					label = "unknown", 
					score = 1, 
					color = {r = 0, g = 0, b = 0, a = 0}, 
					valid = 1 '\001', 
					space_frame = "velodyne", 
					pose = {
						position = {x = 3.7677645683288574, y = -8.3126049041748047, z = -0.97285497188568115}, 
        		orientation = {x = 0, y = 0, z = 0, w = 1}
					}, 
					dimensions = {x = 16.302268981933594, y = 4.062281608581543, z = 2.3455097675323486}, 
					variance = {x = 0, y = 0, z = 0}, 
					velocity = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}, 
					acceleration = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}, 
					pointcloud = {header = {seq = 4405, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494473977, 
              nsec = 362651000}, <No data fields>}, frame_id = "velodyne"}, height = 1, width = 1482, fields = std::vector of length 4, capacity 4 = {{
            name = "x", offset = 0, datatype = 7 '\a', count = 1}, {name = "y", offset = 4, datatype = 7 '\a', count = 1}, {name = "z", offset = 8, 
            datatype = 7 '\a', count = 1}, {name = "rgb", offset = 16, datatype = 7 '\a', count = 1}}, is_bigendian = 0 '\000', point_step = 32, 
						row_step = 47424, data = std::vector of length 47424, capacity 47424 = {207 '\317', 179 '\263', 62 '>', 65 'A', 192 '\300', 1 '\001', 201 '\311', 
							192 '\300', 157 '\235', 190 '\276', 211 '\323', 191 '\277', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 
							0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 167 '\247', 4 '\004', 
							55 '7', 65 'A', 162 '\242', 37 '%', 201 '\311', 192 '\300', 104 'h', 33 '!', 205 '\315', 191 '\277', 0 '\000', 0 '\000', 128 '\200', 63 '?', 
							187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 
							0 '\000', 0 '\000', 156 '\234', 170 '\252', 57 '9', 65 'A', 155 '\233', 191 '\277', 205 '\315', 192 '\300', 252 '\374', 127 '\177', 208 '\320', 
							191 '\277', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 
							22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 224 '\340', 4 '\004', 56 '8', 65 'A', 47 '/', 177 '\261', 210 '\322', 
							192 '\300', 19 '\023', 72 'H', 208 '\320', 191 '\277', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 
							0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 128 '\200', 58 ':', 55 '7', 
							65 'A', 78 'N', 103 'g', 211 '\323', 192 '\300', 74 'J', 200 '\310', 207 '\317', 191 '\277', 0 '\000', 0 '\000', 128 '\200', 63 '?', 187 '\273', 
							81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 240 '\360', 255 '\377', 127 '\177', 0 '\000', 
							0 '\000', 219 '\333', 58 ':', 50 '2', 65 'A', 6 '\006', 226 '\342', 208 '\320', 192 '\300', 87 'W', 234 '\352', 202 '\312', 191 '\277', 0 '\000', 
							0 '\000', 128 '\200', 63 '?', 187 '\273', 81 'Q', 151 '\227', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 0 '\000', 64 '@', 22 '\026', 255 '\377', 
							240 '\360', 255 '\377', 127 '\177', 0 '\000', 0 '\000', 143 '\217', 36 '$', 52 '4', 65 'A', 16 '\020', 231 '\347', 212 '\324', 192 '\300'...}, 
								is_dense = 1 '\001'}, 
						convex_hull = {header = {seq = 4405, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494473977, 
              nsec = 362651000}, <No data fields>}, frame_id = "velodyne"}, polygon = {points = std::vector of length 22, capacity 22 = {{x = 11.9188986, 
              y = -6.28146362, z = -2.14560986}, {x = 11.4386358, y = -6.28584385, z = -2.14560986}, {x = -3.8541646, y = -7.54143143, z = -2.14560986}, {
              x = -3.92919087, y = -7.55756712, z = -2.14560986}, {x = -4.38336992, y = -7.82386923, z = -2.14560986}, {x = -3.69044614, y = -8.95813179, 
              z = -2.14560986}, {x = -2.57893419, y = -9.40120506, z = -2.14560986}, {x = 5.3091197, y = -10.3437452, z = -2.14560986}, {x = 10.6661654, 
              y = -8.86780643, z = -2.14560986}, {x = 11.3075743, y = -7.85019159, z = -2.14560986}, {x = 11.9188986, y = -6.28146362, z = -2.14560986}, {
              x = 11.9188986, y = -6.28146362, z = 0.199899867}, {x = 11.4386358, y = -6.28584385, z = 0.199899867}, {x = -3.8541646, y = -7.54143143,
*/

/*
{header = {seq = 0, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494474087, nsec = 88049764}, <No data fields>}, frame_id = "velodyne"}, 
 id = 0, 
 label = "unknown",
 score = 0, 
 color = {r = 0, g = 0, b = 0, a = 0}, 
 valid = 1 '\001', 
 space_frame = "", 
 pose = {position = {x = 0, y = 0, z = 0}, orientation = {
      x = 0, y = 0, z = 0, w = 0}}, 
dimensions = {x = 0, y = 0, z = 0}, 
variance = {x = 0, y = 0, z = 0}, 
velocity = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, 
      y = 0, z = 0}}, 
acceleration = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}, 
pointcloud = {header = {seq = 0, 
      stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0}, <No data fields>}, frame_id = ""}, height = 0, width = 0, 
    fields = std::vector of length 0, capacity 0, is_bigendian = 0 '\000', point_step = 0, row_step = 0, data = std::vector of length 0, capacity 0, 
    is_dense = 0 '\000'}, 
	convex_hull = {header = {seq = 0, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0}, <No data fields>}, 
      frame_id = ""}, polygon = {points = std::vector of length 0, capacity 0}},
	candidate_trajectories = {id = 0, lanes = std::vector of length 0, capacity 0}, 
  pose_reliable = 0 '\000', 
	velocity_reliable = 0 '\000', 
	acceleration_reliable = 0 '\000', 
	image_frame = "",
	x = 0,
	y = 0, 
	width = 0, 
	height = 0, 
	angle = 0, 
  roi_image = {header = {seq = 0, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0}, <No data fields>}, frame_id = ""}, height = 0, width = 0,
    encoding = "", is_bigendian = 0 '\000', step = 0, data = std::vector of length 0, capacity 0}, indicator_state = 0 '\000', behavior_state = 0 '\000', 
  user_defined_info = std::vector of length 0, capacity 0}
*/

/*
 candidate_trajectories = {id = 0, lanes = std::vector of length 0, capacity 0}, 
 pose_reliable = 0 '\000', velocity_reliable = 0 '\000', 
      acceleration_reliable = 0 '\000', image_frame = "", x = 0, y = 0, width = 0, height = 0, angle = 0, 
roi_image = {header = {seq = 0, 
          stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0}, <No data fields>}, frame_id = ""}, height = 0, width = 0, encoding = "", 
        is_bigendian = 0 '\000', step = 0, data = std::vector of length 0, capacity 0}, 
	indicator_state = 0 '\000', behavior_state = 0 '\000', 
      user_defined_info = std::vector of length 0, capacity 0}, {header = {seq = 5458, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494474086, 
            nsec = 189481000}, <No data fields>}, frame_id = "velodyne"}, id = 0, label = "unknown", score = 1, color = {r = 0, g = 0, b = 0, a = 0}, valid = 1 '\001', 
      space_frame = "velodyne", pose = {position = {x = 0.73513799905776978, y = -12.209321975708008, z = -1.63214111328125}, orientation = {x = 0, y = 0, z = 0, 
          w = 1}}, dimensions = {x = 1.7810214757919312, y = 2.5647516250610352, z = 1.2586542367935181}, variance = {x = 0, y = 0, z = 0}, velocity = {linear = {
          x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}, acceleration = {linear = {x = 0, y = 0, z = 0}, angular = {x = 0, y = 0, z = 0}}, pointcloud = {
        header = {seq = 5458, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 1494474086, nsec = 189481000}, <No data fields>}, frame_id = "velodyne"}, 
        height = 1, width = 28, fields = std::vector of length 4, capacity 4 = {{name = "x", offset = 0, datatype = 7 '\a', count = 1}, {name = "y", offset = 4, 
            datatype = 7 '\a', count = 1}, {name = "z", offset = 8, datatype = 7 '\a', count = 1}, {name = "rgb", offset = 16, datatype = 7 '\a', count = 1}}, 
*/
