#include <communication/TCP_Client.h>
#include <sys/ioctl.h>
#include <communication/TCP.h>

using namespace communication;

TCP_Client::TCP_Client():port((char*)"5050"){
    this->constructor();
}
TCP_Client::TCP_Client(char* port):port(port){
    this->constructor();
}
TCP_Client::~TCP_Client(){
    close(sockFD);
    freeaddrinfo(res);
}

void TCP_Client::run(const ros::TimerEvent& e){

    TCP_ClientState_t oldState = this->aState;

        switch (this->aState){

        case unstated:
            this->aState = this->onUnstated();
            break;

        case init:
            this->aState = this->onInit();
            break;

        case connecting:
            this->aState = this->onConnecting();
            break;

        case connected:
            this->aState = this->onConnected();
            break;

        case connection_lost:
            this->aState = this->onConnection_lost();
            break;

        default:
            ROS_FATAL("TCP_Client has reached undefined state! Please contact the package maintainer!");
            break;
        }

        this->evaluateState(oldState);

        return;

}
bool TCP_Client::sendMsg(SendTCP::Request& req, SendTCP::Response& res){
        
    ssize_t bytes_sent = -1;
    char msg[15];

    convertMsg_mtc(req, msg);    

    ROS_INFO("sending MSG");

    if(alive){
    
        bytes_sent = send(sockFD, msg, 15, 0);

    }

    if(bytes_sent == -1) alive = false;

    res.send = alive;

    return res.send;

}

void TCP_Client::constructor(){

    aState = unstated;

    alive = false;

    ros::NodeHandle nh;

    nh.param("ipAddress", this->ipAddress, (STRING)"192.168.2.120"); 

    this->msg_pub = nh.advertise<SendTCP::Request>("com_recv_msg", 10);
    this->state_pub = nh.advertise<std_msgs::String>("TCP_Client_state", 10);
    this->sendMsgService = nh.advertiseService("com_send_msg",&TCP_Client::sendMsg,this);

    this->run_timer_ = nh.createTimer(ros::Duration(0.1), &TCP_Client::run, this, false );
    this->run_timer_.start();

}
TCP_ClientState_t TCP_Client::onUnstated(){
    return init;
}
TCP_ClientState_t TCP_Client::onInit(){

    addrinfo hints;
    memset(&hints, 0, sizeof(hints));

    hints.ai_family   = AF_UNSPEC;    // don't specify which IP version to use yet
    hints.ai_socktype = SOCK_STREAM;  // SOCK_STREAM refers to TCP, SOCK_DGRAM will be?
    hints.ai_flags    = AI_PASSIVE;

    int gAddRes = getaddrinfo(ipAddress.c_str(), this->port, &hints, &res);
    if (gAddRes != 0) {

        std::ostringstream stm ;
        stm << gai_strerror(gAddRes);

        const char* err = stm.str().c_str();

        ROS_ERROR(err);
        
        return init;

    }


    // if no addresses found
    if (res == NULL) {
        ROS_ERROR("No addresses found");
        return init;
    }

    // socket() call creates a new socket and returns it's descriptor
    sockFD = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (sockFD == -1) {
        ROS_ERROR("Error while creating socket");
        freeaddrinfo(res);
        return init;
    }

    alive = true;

    return connecting;

}
TCP_ClientState_t TCP_Client::onConnecting(){

    // connect() call tries to establish a TCP connection to the specified server
    connectR = connect(sockFD, res->ai_addr, res->ai_addrlen);
    if (connectR == -1) {
        close(sockFD);
        ROS_ERROR("Error while connecting socket");
        return connection_lost;
    }

    return connected;

}
TCP_ClientState_t TCP_Client::onConnected(){

    if(!alive) return connection_lost;

    char reply[15];

    int bytesAvailable;
    ioctl(sockFD, FIONREAD, &bytesAvailable);

    if(bytesAvailable){

        int msgLen = recv(sockFD, &reply, 15, 0);

        if(msgLen ==-1) return connection_lost;
        if(msgLen ==0) return connection_lost;

        msg_pub.publish(convertMsg_ctm(reply));
        //DEBUG
        ROS_INFO(reply);

    }

    return connected;

}

TCP_ClientState_t TCP_Client::onConnection_lost(){

    close(sockFD);
    freeaddrinfo(res);
    return init;

}
void TCP_Client::evaluateState(TCP_ClientState_t oldState){

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
    case connecting:
        state.data = "connecting";
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