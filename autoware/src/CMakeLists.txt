set(autoware_SRCS
   AutowareServiceReceiver.cpp
)
add_executable (autoware_receiver ${autoware_SRCS})
target_link_libraries (autoware_receiver zmq protobuf boost_system boost_thread boost_serialization asn proto messages utility uci)
