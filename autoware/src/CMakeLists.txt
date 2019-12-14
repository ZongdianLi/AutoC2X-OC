set(autoware_SRCS
   AutowareService.cpp
)
include_directories(/home/oi/Downloads/cereal-1.3.0/include)
add_executable (autoware ${autoware_SRCS})
target_link_libraries (autoware zmq protobuf boost_system boost_thread boost_serialization asn proto messages utility uci)
