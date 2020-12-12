#include <communication/TCP.h>
#include <communication/TCP_Server.h>
#include <sys/ioctl.h>

using namespace communication;

TCP_Server::TCP_Server():port((char*)"5050"){
    this->constructor();
}
TCP_Server::TCP_Server(char* port):port(port){
    this->constructor();
}
TCP_Server::~TCP_Server(){
    close(newFD);
    close(sockFD);
    freeaddrinfo(res);
}

void TCP_Server::constructor(){

    aState = unstated;

    alive = false;

    ros::NodeHandle nh;

    nh.param("backLog", this->backLog, (int)1); 

    this->msg_pub = nh.advertise<SendTCP::Request>("com_recv_msg", 10);
    this->state_pub = nh.advertise<std_msgs::String>("TCP_Server_state", 10);
    this->sendMsgService = nh.advertiseService("com_send_msg",&TCP_Server::sendMsg,this);

    this->run_timer_ = nh.createTimer(ros::Duration(0.1), &TCP_Server::run, this, false );
    this->run_timer_.start();

}

TCP_ServerState_t TCP_Server::onUnstated(){
    return init;
}

TCP_ServerState_t TCP_Server::onInit(){

    addrinfo hints;
    memset(&hints, 0, sizeof(hints));

    hints.ai_family   = AF_UNSPEC;    // don't specify which IP version to use yet
    hints.ai_socktype = SOCK_STREAM;  // SOCK_STREAM refers to TCP, SOCK_DGRAM will be?
    hints.ai_flags    = AI_PASSIVE;

    int gAddRes = getaddrinfo(NULL, this->port, &hints, &res);
    if (gAddRes != 0) {

        std::ostringstream stm ;
        stm << gai_strerror(gAddRes);

        const char* err = stm.str().c_str();

        ROS_ERROR(err);
        
        return init;

    }

    ROS_INFO("Detecting addresses");

    // if no addresses found
    if (res == NULL) {
        ROS_ERROR("Found no host address to use");
        return init;
    }

    char ipStr[INET6_ADDRSTRLEN];    // ipv6 length makes sure both ipv4/6 addresses can be stored in this variable

    // Now since getaddrinfo() has given us a list of addresses
    // we're going to choose the fist
    void *addr;
    std::string ipVer;

    // if address is ipv4 address
    if (res->ai_family == AF_INET) {
        ipVer             = "IPv4";
        sockaddr_in *ipv4 = reinterpret_cast<sockaddr_in *>(res->ai_addr);
        addr              = &(ipv4->sin_addr);
    }

    // if address is ipv6 address
    else {
        ipVer              = "IPv6";
        sockaddr_in6 *ipv6 = reinterpret_cast<sockaddr_in6 *>(res->ai_addr);
        addr               = &(ipv6->sin6_addr);
    }

    // convert IPv4 and IPv6 addresses from binary to text form
    inet_ntop(res->ai_family, addr, ipStr, sizeof(ipStr));
    std::string s = ipVer + " : " + ipStr;

    //ROS_INFO(s.c_str());

    // Create a new socket, socketFD is returned as descriptor
    // these calls usually return -1 as result of some error
    sockFD = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sockFD == -1) {
        ROS_ERROR("Error while creating socket");
        freeaddrinfo(res);
        return init;
    }

    // Bind address to our socket we've just created
    int bindR = bind(sockFD, res->ai_addr, res->ai_addrlen);
    if (bindR == -1) {
        ROS_ERROR("Error while binding socket");
        
        // if some error occurs, make sure to close socket and free resources
        close(sockFD);
        freeaddrinfo(res);
        return init;
    }

    // Start listening for connections on our socket
    int listenR = listen(sockFD, backLog);
    if (listenR == -1) {
        ROS_ERROR("Error while Listening on socket");

        // if some error occurs, make sure to close socket and free resources
        close(sockFD);
        freeaddrinfo(res);
        return init;
    }

    
    // structure large enough to hold client's address
    
    client_addr_size = sizeof(client_addr);

    alive = true;

    return listening;

}

TCP_ServerState_t TCP_Server::onListening(){

    // accept call will give us a new socket descriptor
    newFD = accept(sockFD, (sockaddr *) &client_addr, &client_addr_size);

    if (newFD == -1) {
        ROS_ERROR("Error while Accepting on socket");
        return listening;
    }

    return connected;
}

TCP_ServerState_t TCP_Server::onConnected(){

    if(!alive) return connection_lost;

    char reply[15];

    int bytesAvailable;
    ioctl(newFD, FIONREAD, &bytesAvailable);

    if(bytesAvailable){

        int msgLen = recv(newFD, &reply, 15, 0);

        if(msgLen ==-1) return connection_lost;
        if(msgLen ==0) return connection_lost;

        msg_pub.publish(convertMsg_ctm(reply));
        //DEBUG
        ROS_INFO(reply);

    }

    return connected;

}

TCP_ServerState_t TCP_Server::onConnection_lost(){

    close(newFD);
    close(sockFD);
    freeaddrinfo(res);
    return init;

}

void TCP_Server::run(const ros::TimerEvent& e){
    
    TCP_ServerState_t oldState = this->aState;

        switch (this->aState){

        case unstated:
            this->aState = this->onUnstated();
            break;

        case init:
            this->aState = this->onInit();
            break;

        case listening:
            this->aState = this->onListening();
            break;

        case connected:
            this->aState = this->onConnected();
            break;

        case connection_lost:
            this->aState = this->onConnection_lost();
            break;

        default:
            ROS_FATAL("TCP_Server has reached undefined state! Please contact the package maintainer!");
            break;
        }

        this->evaluateState(oldState);

        return;

}

void TCP_Server::evaluateState(TCP_ServerState_t oldState){
    
    if(oldState == this->aState) return;

    std_msgs::String state;

    switch (this->aState)
    {
    case unstated:
        state.data = "unstated";
        break;
    case init:
        state.data = "init";
        break;
    case listening:
        state.data = "listening";
        break;
    case connected:
        state.data = "connected";
        break;
    case connection_lost:
        state.data = "connection_lost";
        break;
    
    default:
        break;
    }

    //Debug
    ROS_INFO("State: %s", state.data.c_str());

    this->state_pub.publish(state);

    return;
    
}

bool TCP_Server::sendMsg(SendTCP::Request& req, SendTCP::Response& res){

    ssize_t bytes_sent = -1;
    char msg[15];

    convertMsg_mtc(req, msg);    

    ROS_INFO("sending MSG");

    if(alive){
    
        bytes_sent = send(newFD, msg, 15, 0);

    }

    if(bytes_sent == -1) alive = false;

    res.send = alive;

    return res.send;

}