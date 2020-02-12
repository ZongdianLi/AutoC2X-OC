set(cam_SRCS
   caserviceSender.cpp
)

add_executable (cam_sender ${cam_SRCS})
target_link_libraries (cam_sender pthread zmq protobuf boost_system boost_thread asn proto messages utility uci)
install(TARGETS cam_sender DESTINATION bin)
