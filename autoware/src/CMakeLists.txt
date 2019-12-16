set(autoware_SRCS
   AutowareService.cpp
)
add_executable (autoware ${autoware_SRCS})
target_link_libraries (autoware zmq protobuf boost_system boost_thread boost_serialization asn proto messages utility uci)
