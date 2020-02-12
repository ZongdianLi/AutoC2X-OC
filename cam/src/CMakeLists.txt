set(cam_SRCS
   caserviceReceiver.cpp
)

add_executable (cam_receiver ${cam_SRCS})
target_link_libraries (cam_receiver pthread zmq protobuf boost_system boost_thread asn proto messages utility uci)
install(TARGETS cam_receiver DESTINATION bin)
