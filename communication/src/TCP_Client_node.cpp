#include <ros/ros.h>
#include <communication/TCP_Client.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ROS_INFO("%s started.", ROS_PACKAGE_NAME);

    communication::TCP_Client client(argv[1]);

    ros::spin();
    return 0;

}