#include <ros/ros.h>
#include <communication/TCP_Server.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ROS_INFO("%s started.", ROS_PACKAGE_NAME);

    communication::TCP_Server server;

    ros::spin();
    return 0;

}