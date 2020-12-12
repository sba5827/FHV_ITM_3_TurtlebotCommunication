#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "ros/ros.h"

#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <std_msgs/String.h>
#include <communication/SendTCP.h>

#ifndef STRING
#define STRING std::__cxx11::string
#endif //STRING

namespace communication
{
    typedef enum {unstated, init, connecting, connected, connection_lost} TCP_ClientState_t;

    class TCP_Client
    {
    protected:
        ros::Publisher state_pub;
        ros::Publisher msg_pub;
        ros::ServiceServer sendMsgService;
        ros::Timer run_timer_;

        TCP_ClientState_t aState;

        //SocketSpecific
        char* port;
        STRING ipAddress;
        addrinfo *res;
        int sockFD;
        int connectR;
        bool alive;

        void constructor();
        TCP_ClientState_t onUnstated();
        TCP_ClientState_t onInit();
        TCP_ClientState_t onConnecting();
        TCP_ClientState_t onConnected();
        TCP_ClientState_t onConnection_lost();
        void evaluateState(TCP_ClientState_t oldState);
      

    public:
        TCP_Client();
        TCP_Client(char* port);
        ~TCP_Client();

        void run(const ros::TimerEvent& e);
        bool sendMsg(SendTCP::Request& req, SendTCP::Response& res);
    };
    
     
} // namespace communication


#endif //TCP_CLIENT_H