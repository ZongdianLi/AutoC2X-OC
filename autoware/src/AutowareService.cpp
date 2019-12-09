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




void init(ros::NodeHandle n) {
	paramOrganize("proj=tmerc lat_0=36 lon_0=139.8333333333333 k=0.9999 x_0=0 y_0=0 ellps=GRS80 units=m");

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

	filename = std::string(cur_dir) + "/../autoware/output/1_2_delay/" + timestamp + ".csv";
	std::cout << "filename:" << filename << std::endl;
	one_two_delay_file.open(filename, std::ios::out);

	//通信モードの時は使う
	struct sockaddr_in addr;
	if( (sockfd = socket( AF_INET, SOCK_STREAM, 0) ) < 0 ) perror( "socket" ); 
	addr.sin_family = AF_INET;
	addr.sin_port = htons( 23457 );
	addr.sin_addr.s_addr = inet_addr( "192.168.1.1" );
	connect( sockfd, (struct sockaddr *)&addr, sizeof( struct sockaddr_in ) );

}


void sendToRouter(){
	 // データ送信
	char send_str[10];
	char receive_str[10];
	message message;
	message.speed = speed * 100;
	message.time =  ((generationUnixTimeSec*1000 + (int)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536;
	message.longitude = longitude * 10000000;
	message.latitude = latitude * 10000000;
	
	s_message.timestamp = Utils::currentTime();
	s_message.speed.push_back(speed * 100);
	s_message.time.push_back(((generationUnixTimeSec*1000 + (int)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536);
	s_message.longitude.push_back(longitude * 10000000);
	s_message.latitude.push_back(latitude * 10000000);

	std::cout << "delay: " <<  s_message.timestamp << std::endl;

	std::stringstream ss;
	boost::archive::text_oarchive archive(ss);
	archive << s_message;

	ss.seekg(0, ios::end);
	if( send( sockfd, ss.str().c_str(), ss.tellp(), 0 ) < 0 ) {
			perror( "send" );
	} else {
	}
}

std::string paramOrganize(std::string param){
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

double calcSpeed(){
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

void timeCalc(){
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

void callback(const geometry_msgs::PoseStamped msg){
	prevPose = nowPose;
	nowPose = msg;
	timeCalc();
	speed = calcSpeed();
	sendToRouter();
}

void callback_objects(const autoware_msgs::DetectedObjectArray msg){
	// std::cout << "---------" << std::endl;
	s_message.longitude.clear();
	s_message.latitude.clear();
	s_message.speed.clear();
	s_message.time.clear();
	// s_message.data.clear();
	for(int i = 0; i < msg.objects.size(); i++){
		message s;
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

		s_message.longitude.push_back(result.u * 10000000);
		s_message.latitude.push_back(result.v * 10000000);
		s_message.speed.push_back(0);
		s_message.time.push_back(((generationUnixTimeSec*1000 + (int)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536);

		// std::cout << std::setprecision(20) <<  "lat:" << result.v * 10000000 << " lon:" << result.u * 10000000 << " time:" << (((long)generationUnixTimeSec*1000 + (long)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536 << std::endl;
		// s.longitud e = result.u;
		// s.latitude = result.v;
		// s.speed = 0;
		// s.time = ((generationUnixTimeSec*1000 + (int)generationUnixTimeNSec/1000000 - 1072850400000)) % 65536;
		// s_message.data.push_back(s);
	}
	// sendToRouter();
}

void sampleCallback(autoware_msgs::DetectedObjectArray msg){
	std::cout << "listen:" << msg.header.stamp.sec << std::endl; 
}

void receiveFromRouter(){
		std::cout << "*****receive setup" << std::endl;
		int sockfd;
    int client_sockfd;
    struct sockaddr_in addr;
    socklen_t len = sizeof( struct sockaddr_in );
    struct sockaddr_in from_addr;
    char buf[4096];
 
    memset( buf, 0, sizeof( buf ) );
    if( ( sockfd = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) {
        perror( "socket" );
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons( 23458 );
    addr.sin_addr.s_addr = INADDR_ANY;
    if( bind( sockfd, (struct sockaddr *)&addr, sizeof( addr ) ) < 0 ) perror( "bind" );
    if( listen( sockfd, SOMAXCONN ) < 0 ) perror( "listen" );
    if( ( client_sockfd = accept( sockfd, (struct sockaddr *)&from_addr, &len ) ) < 0 ) perror( "accept" );
 
    // 受信
    int rsize;
    while( 1 ) {
		message message;
		// socket_message s_message;
		std::stringstream ss;
		memset( buf, 0, sizeof( buf ) );
        rsize = recv( client_sockfd, buf, sizeof( buf ), 0 );
		ss << buf;

		boost::archive::text_iarchive archive(ss);
		archive >> s_message;
		one_two_delay_file << Utils::currentTime() << "," << s_message.timestamp << std::endl;
        if ( rsize == 0 ) {
            break;
        } else if ( rsize == -1 ) {
            perror( "recv" );
        }
    }
    close( client_sockfd );
    close( sockfd );

}


int main(int argc,  char* argv[]) {
		mThreadReceiveFromRouter = new boost::thread(boost::ref(receiveFromRouter));

	ros::init(argc, argv, "listener");
	ros::NodeHandle n,n2;

	ros::Subscriber sub1 = n.subscribe("ndt_pose", 1024, callback);
	ros::Subscriber sub2 = n.subscribe("detection/lidar_detector/objects", 1024, callback_objects);
	init(n);

	std::cout << "hairuyo" << std::endl;
	ros::spin();
	std::cout << "detayo" << std::endl;
	return 0;
}
