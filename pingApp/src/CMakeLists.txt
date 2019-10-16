include_directories(
    ${PROJECT_SOURCE_DIR}
)
set(pingApp_SRCS
   pingApp.cpp
)

add_executable (pingApp ${pingApp_SRCS})
target_link_libraries (pingApp zmq protobuf boost_system boost_thread pthread asn proto messages utility uci)

install(TARGETS pingApp DESTINATION bin)
