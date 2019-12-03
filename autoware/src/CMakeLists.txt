set(autoware_SRCS
   AutowareService.cpp
   SerialPort.cpp
)

include_directories(/opt/ros/kinetic/include)
include_directories(/home/oi/Autoware/ros/install/autoware_msgs/include)
link_directories(/opt/ros/kinetic/lib)
add_executable (autoware ${autoware_SRCS})
target_link_libraries (autoware zmq protobuf boost_system boost_thread boost_serialization asn proto messages utility roscpp rosconsole rostime roscpp_serialization proj uci)
