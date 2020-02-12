set(autoware_SRCS
   AutowareServiceSender.cpp
)

add_executable (autoware_sender ${autoware_SRCS})
target_link_libraries (autoware_sender zmq protobuf boost_system boost_thread asn proto messages utility uci boost_serialization)
