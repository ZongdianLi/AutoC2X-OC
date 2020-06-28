set(cam_SRCS
   mcserviceSender.cpp
)

add_executable (mcm_sender ${mcm_SRCS})
target_link_libraries (mcm_sender pthread zmq protobuf boost_system boost_thread asn proto messages utility uci)
install(TARGETS mcm_sender DESTINATION bin)

set(mcm_receiver_SRCS
   mcserviceReceiver.cpp
)

add_executable (mcm_receiver ${mcm_receiver_SRCS})
target_link_libraries (mcm_receiver pthread zmq protobuf boost_system boost_thread asn proto messages utility uci)
install(TARGETS mcm_receiver DESTINATION bin)
