set(cam_SRCS
   caserviceSender.cpp
)

add_executable (cam_sender ${cam_SRCS})
target_link_libraries (cam_sender pthread zmq protobuf boost_system boost_thread asn proto messages utility uci)
install(TARGETS cam_sender DESTINATION bin)

set(cam_receiver_SRCS
   caserviceReceiver.cpp
)

add_executable (cam_receiver ${cam_receiver_SRCS})
target_link_libraries (cam_receiver pthread zmq protobuf boost_system boost_thread asn proto messages utility uci)
install(TARGETS cam_receiver DESTINATION bin)
