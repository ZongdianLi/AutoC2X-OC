set(pingApp_SRCS
   pingApp.cpp
)

add_executable (pingApp ${pingApp_SRCS})
target_link_libraries (pingApp zmq protobuf sqlite3 boost_system boost_thread asn proto messages utility uci)

install(TARGETS pingApp DESTINATION bin)