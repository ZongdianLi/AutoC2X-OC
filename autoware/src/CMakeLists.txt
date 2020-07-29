set(autoware_SRCS
   AutowareServiceSender.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

include_directories("/home/parallels/rosbridgecpp" "/home/parallels/json11")
add_executable (autoware_sender ${autoware_SRCS})
target_link_libraries (autoware_sender zmq protobuf boost_system boost_thread asn proto messages utility uci boost_serialization ssl crypto)
# set(autoware_receiver_SRCS
#    AutowareServiceReceiver.cpp
# )
# add_executable (autoware_receiver ${autoware_receiver_SRCS})
# target_link_libraries (autoware_receiver zmq protobuf boost_system boost_thread boost_serialization asn proto messages utility uci)
