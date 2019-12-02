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
float AutowareService::generationUnixTime;
long AutowareService::generationUnixTimeSec;
long AutowareService::generationUnixTimeNSec;
std::vector<message> AutowareService::message_arr;
geometry_msgs::PoseStamped AutowareService::nowPose;
geometry_msgs::PoseStamped AutowareService::prevPose;
std::ofstream AutowareService::delay_output_file;
int AutowareService::sockfd;

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

	// std::string filename = std::string(cur_dir) + "/../../../autoware/output/delay/" + timestamp + ".csv";
	std::string filename = std::string(cur_dir) + "/../autoware/output/delay/" + timestamp + ".csv";
	std::cout << "filename:" << filename << std::endl;
	delay_output_file.open(filename, std::ios::out);


	std::cout << "hello" << std::endl;

	//通信モードの時は使う
	// struct sockaddr_in addr;
	// if( (sockfd = socket( AF_INET, SOCK_STREAM, 0) ) < 0 ) perror( "socket" ); 
	// addr.sin_family = AF_INET;
	// addr.sin_port = htons( 23457 );
	// addr.sin_addr.s_addr = inet_addr( "192.168.1.1" );
	// connect( sockfd, (struct sockaddr *)&addr, sizeof( struct sockaddr_in ) );


}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	mTimer->cancel();
	delete mTimer;
}

void AutowareService::sendToRouter(){
	 // データ送信
    char send_str[10];
    char receive_str[10];
		message message;
		message.speed = speed * 100;
		message.time =  ((generationUnixTimeSec*1000 + (int)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536;
		message.longitude = longitude * 10000000;
		message.latitude = latitude * 10000000;
		message_arr.push_back(message);
		// std::cout << "generationDelta:" <<  (long)generationUnixTime << std::endl;
		char* my_s_bytes = static_cast<char*>(static_cast<void*>(&message_arr));
		if( send( sockfd, my_s_bytes, sizeof(message), 0 ) < 0 ) {
				perror( "send" );
		} else {
		}
}

//reads the actual vehicle data from Autoware
void AutowareService::receiveData(const boost::system::error_code &ec, SerialPort* serial) {
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
	// autoware.set_time(Utils::currentTime());
	autoware.set_time(generationUnixTime * 1000000000);

	autoware.set_longitude(longitude * 10000000);
	autoware.set_latitude(latitude * 10000000);

	
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
	paramOrganize("proj=tmerc lat_0=36 lon_0=139.8333333333333 k=0.9999 x_0=0 y_0=0 ellps=GRS80 units=m");
	n = &tmp;
	if (!mConfig.mSimulateData) {	//use real Autoware data
	}
	else {				//use simulated Autoware data
	}

}

double AutowareService::calcSpeed(){
		double prevTime = prevPose.header.stamp.sec + prevPose.header.stamp.nsec/1000000000.0;
		double nowTime = nowPose.header.stamp.sec + nowPose.header.stamp.nsec/1000000000.0;
		double timedelta = nowTime - prevTime;

		double d_x = pow(prevPose.pose.position.x - nowPose.pose.position.x, 2);
		double d_y = pow(prevPose.pose.position.y - nowPose.pose.position.y, 2);

		// double degToRad = M_PI / 180;
		projUV xy;
		xy.u = nowPose.pose.position.x;
		xy.v = nowPose.pose.position.y;
		projUV result = pj_inv(xy, p_proj);
		result.u /= DEG_TO_RAD;
		result.v /= DEG_TO_RAD;
		std::cout << std::setprecision(20) << result.v << "," << result.u << std::endl;

		longitude = result.u;
		latitude = result.v;

		return sqrt(d_x + d_y) / timedelta;

}

void AutowareService::timeCalc(){
	geometry_msgs::PoseStamped newestPose = nowPose;
	float messageRosTime = newestPose.header.stamp.sec +  newestPose.header.stamp.nsec / 1000000000.0;
	// double diffTimeFromRosToWall = (ros::WallTime::now().toSec() - ros::Time::now().toSec() - ros::Time::now().toSec() + ros::WallTime::now().toSec()) / 2.0;
	ros::Time  a2, a3;
	ros::WallTime a1, a4;
	a1 = ros::WallTime::now();
	a2 = ros::Time::now();
	// a3 = ros::Time::now();
	// a4 = ros::WallTime::now();
	long diffTimeSec = ((long)a1.sec - (long)a2.sec);
	long diffTimeNSec = ((long)a1.nsec - (long)a2.nsec);
	// float diffTimeFromRosToWall = diffTimeSec + diffTimeNSec / 1000000000.0;
	// std::cout << std::setprecision(20) << std::endl;
	// std::cout << "rostime: " << a1.sec << "." << a1.nsec << std::endl;
	// std::cout << "walltime: " << a2.sec << "." << a2.nsec << std::endl;
	// std::cout << "Sec=" << diffTimeSec << " nsec=" << diffTimeNSec << std::endl;

	
	generationUnixTimeSec = (long)newestPose.header.stamp.sec + diffTimeSec;
	generationUnixTimeNSec = (long)newestPose.header.stamp.nsec + diffTimeNSec;
	if(generationUnixTimeNSec < 0){
		generationUnixTimeSec -= 1;
		generationUnixTimeNSec = 1000000000 + generationUnixTimeNSec;
	}
	if(generationUnixTimeNSec >= 1000000000){
		generationUnixTimeSec += 1;
		generationUnixTimeNSec -= 1000000000;
	}
	// std::cout << "a1.nsec:" << a1.nsec <<  " generationNSec:" << generationUnixTimeNSec << std::endl;
	long delaySec = a1.sec - generationUnixTimeSec;
	long delayNSec = a1.nsec - generationUnixTimeNSec;
	if(delayNSec < 0){
		// std::cout << "hello" << std::endl;
		// std::cout << "sec=" << delaySec << " nsec=" << delayNSec << std::endl;
		delaySec -= 1;
		delayNSec = 1000000000 + delayNSec;
	}
	delay_output_file <<  std::setprecision(20) <<  ros::WallTime::now() << "," << delayNSec / 1000000000.0 << std::endl;
	// std::cout << "delay:" << delayNSec/ 1000000000.0 << " a1.nsec:" << a1.nsec << " generationNSec:" << generationUnixTimeNSec << std::endl;
	// std::cout << "generationUnixTime:" <<  std::setprecision(20) << generationUnixTimeSec << "." << generationUnixTimeNSec << std::endl;
}

void AutowareService::callback(const geometry_msgs::PoseStamped msg){

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

	prevPose = nowPose;
	nowPose = msg;
	// const autoware_msgs::DetectedObjectArray message = msg;
	timeCalc();
	speed = calcSpeed();
	// printf("%f\n", speed);
	// simulateData();
	// sendToRouter();
}

void AutowareService::callback_objects(const autoware_msgs::DetectedObjectArray msg){
	std::cout << "---------" << std::endl;
	message_arr.clear();
	for(int i = 0; i < msg.objects.size(); i++){
		message sock_msg;
		float sum_x = 0.0;
		float sum_y = 0.0;
		float sum_z = 0.0;
		for(int j = 0; j < msg.objects[i].convex_hull.polygon.points.size(); j++){
			sum_x += msg.objects[i].convex_hull.polygon.points[j].x;
			sum_y += msg.objects[i].convex_hull.polygon.points[j].y;
			sum_z += msg.objects[i].convex_hull.polygon.points[j].z;
		}
		sum_x /= (float)msg.objects[i].convex_hull.polygon.points.size();
		sum_y /= (float)msg.objects[i].convex_hull.polygon.points.size();
		sum_z /= (float)msg.objects[i].convex_hull.polygon.points.size();

		projUV xy;
		xy.u = sum_x;
		xy.v = sum_y;
		projUV result = pj_inv(xy, p_proj);
		result.u /= DEG_TO_RAD;
		result.v /= DEG_TO_RAD;
		// std::cout << std::setprecision(20) << result.v << "," << result.u << std::endl;

		sock_msg.longitude = result.u;
		sock_msg.latitude = result.v;
		sock_msg.speed = 0;
		sock_msg.time =((generationUnixTimeSec*1000 + (int)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536;

		message_arr.push_back(sock_msg);
	}
}

void AutowareService::sampleCallback(autoware_msgs::DetectedObjectArray msg){
	std::cout << "listen:" << msg.header.stamp.sec << std::endl; 
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

ros::Publisher pub;
void createObjectsPublisher(const ros::TimerEvent&){
// void createObjectsPublisher(){
	// ros::NodeHandle n;	
	std::cout << "hello" << std::endl;
	// ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("points_cluster", 1000);

	autoware_msgs::DetectedObjectArray msg;
	std::vector<autoware_msgs::DetectedObject> objects;

	// ros::Time nowTime = ros::Time::now();
	// nowTime.sec = nowTime.sec + 1;

	// msg.header.stamp = nowTime;
	msg.header.frame_id = "velodyne";

	autoware_msgs::DetectedObject object;
	// object.header.stamp = nowTime;
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
	// input_msg.header.stamp = nowTime;
	input_msg.header.frame_id = "velodyne";
	input_msg.points = points;
	input_msg.channels = channels;
	sensor_msgs::convertPointCloudToPointCloud2(input_msg, output_msg);
	object.pointcloud = output_msg;

	geometry_msgs::PolygonStamped convex_hull_msg;
	// convex_hull_msg.header.stamp = nowTime;
	convex_hull_msg.header.frame_id = "velodyne";
	geometry_msgs::Polygon polygon;
	polygon.points = createConvexHull();
	convex_hull_msg.polygon = polygon;
	object.convex_hull = convex_hull_msg;

	objects.push_back(object);
	msg.objects = objects;
	pub.publish(msg);

	// printf("published %d.%d\n",nowTime.sec, nowTime.nsec);
}

void createPointsPublisher(const ros::TimerEvent&){
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("points_cluster", 1000);

	// ros::Rate loop_rate(0.1);

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
	ros::NodeHandle n,n2;

	// ros::Subscriber sub = n.subscribe("ndt_pose", 1024, autoware.callback);
	ros::Subscriber sub = n.subscribe("detection/lidar_detector/objects", 1024, autoware.callback_objects);
	autoware.init(n);

	std::cout << "hairuyo" << std::endl;
	ros::spin();
	std::cout << "detayo" << std::endl;
	return 0;
}
