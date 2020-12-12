#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "ros/ros.h"

#include <cstring>    // sizeof()
#include <iostream>
#include <string> 

// headers for socket(), getaddrinfo() and friends
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <unistd.h>    // close()

#include <std_msgs/String.h>
#include <communication/SendTCP.h>

namespace communication
{
    typedef enum {unstated, init, listening, connected, connection_lost} TCP_ServerState_t;

    class TCP_Server
    {
    protected:
        ros::Publisher state_pub;
        ros::Publisher msg_pub;
        ros::ServiceServer sendMsgService;
        ros::Timer run_timer_;

        TCP_ServerState_t aState;

        //SocketSpecific
        char* port ;
        addrinfo *res;
        sockaddr_storage client_addr;
        socklen_t client_addr_size;
        int sockFD;
        int newFD;
        int backLog;
        bool alive;

        void constructor();
        TCP_ServerState_t onUnstated();
        TCP_ServerState_t onInit();
        TCP_ServerState_t onListening();
        TCP_ServerState_t onConnected();
        TCP_ServerState_t onConnection_lost();
        void evaluateState(TCP_ServerState_t oldState);
      

    public:
        TCP_Server();
        TCP_Server(char* port);
        ~TCP_Server();

        void run(const ros::TimerEvent& e);
        bool sendMsg(SendTCP::Request& req, SendTCP::Response& res);
    };
    
     
} // namespace communication


#endif //TCP_SERVER_H