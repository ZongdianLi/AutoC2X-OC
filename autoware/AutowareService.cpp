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

	// mSender = new CommunicationSender("25000", *mLogger);
    std::cout << "******hey:" <<  mConfig.isReceiver << std::endl;

	 
	
	// char cur_dir[1024];
	// getcwd(cur_dir, 1024);

	// time_t t = time(nullptr);
	// const tm* lt = localtime(&t);
	// std::stringstream s;
	// s<<"20";
	// s<<lt->tm_year-100; //100を引くことで20xxのxxの部分になる
	// s<<"-";
	// s<<lt->tm_mon+1; //月を0からカウントしているため
	// s<<"-";
	// s<<lt->tm_mday; //そのまま
	// s<<"_";
	// s<<lt->tm_hour;
	// s<<":";
	// s<<lt->tm_min;
	// s<<":";
	// s<<lt->tm_sec;
	// std::string timestamp = s.str();

	// std::string filename = std::string(cur_dir) + "/../../../autoware/output/delay/" + timestamp + ".csv";
	// delay_output_file.open(filename, std::ios::out);

}

AutowareService::~AutowareService() {
	delete mSender;
	delete mLogger;

	// mTimer->cancel();
	// delete mTimer;
}

void AutowareService::messageReceive(message message){
	int sockfd;
    int client_sockfd;
    struct sockaddr_in addr;
    socklen_t len = sizeof( struct sockaddr_in );
    struct sockaddr_in from_addr;
    char buf[1024];
 
    memset( buf, 0, sizeof( buf ) );
    if( ( sockfd = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) {
        perror( "socket" );
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons( 23457 );
    addr.sin_addr.s_addr = INADDR_ANY;
    if( bind( sockfd, (struct sockaddr *)&addr, sizeof( addr ) ) < 0 ) perror( "bind" );
    if( listen( sockfd, SOMAXCONN ) < 0 ) perror( "listen" );
    if( ( client_sockfd = accept( sockfd, (struct sockaddr *)&from_addr, &len ) ) < 0 ) perror( "accept" );
 
    // 受信
    int rsize;
    while( 1 ) {
        rsize = recv( client_sockfd, buf, sizeof( buf ), 0 );

		memcpy(&message, buf, sizeof(message));
		std::cout << "received speed:" <<  message.speed << std::endl;

        if ( rsize == 0 ) {
            break;
        } else if ( rsize == -1 ) {
            perror( "recv" );
        }
    }
 
    close( client_sockfd );
    close( sockfd );
}

void AutowareService::messageSend(message message){
	int sockfd;
    struct sockaddr_in addr;
 
    // ソケット生成
    if( (sockfd = socket( AF_INET, SOCK_STREAM, 0) ) < 0 ) perror( "socket" ); 
 
    // 送信先アドレス・ポート番号設定
    addr.sin_family = AF_INET;
    addr.sin_port = htons( 23457 );
    addr.sin_addr.s_addr = inet_addr( "127.0.0.1" );
 
    // サーバ接続
    connect( sockfd, (struct sockaddr *)&addr, sizeof( struct sockaddr_in ) );
 
    // データ送信
    char send_str[10];
    char receive_str[10];
    for ( int i = 0; i < 10; i++ ){
        sprintf( send_str, "%d", i );
		message.speed = i + 10000;
		char* my_s_bytes = static_cast<char*>(static_cast<void*>(&message));
        if( send( sockfd, my_s_bytes, sizeof(message), 0 ) < 0 ) {
            perror( "send" );
        } else {
        }
        sleep( 1 );
    }
    close( sockfd );
}

int main(int argc,  char* argv[]) {


	AutowareConfig config;
	message message;
	try {
		config.loadConfig();
	}
	catch (std::exception &e) {
		cerr << "Error while loading config.xml: " << e.what() << endl << flush;
		return EXIT_FAILURE;
	}
	AutowareService autoware(config);

	
	if(argc == 2){
		autoware.messageSend(message);
	} else{
		autoware.messageReceive(message);
	}

	return 0;
}