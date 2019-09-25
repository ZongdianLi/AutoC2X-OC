#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
void callback(const geometry_msgs::PoseStamped msg){

    int a = 0;

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ndt_pose", 1000, callback);

    ros::spin();

    return 0;
}
