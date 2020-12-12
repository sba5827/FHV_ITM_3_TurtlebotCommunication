# FHV_ITM_3_TurtlebotCommunication
## Install:
Build the package, and do not forget the source code of the devel/setup.bash
Use only one of the following: either the server or the client. The other machine should of course be running the opposite side.

## Start the TCP_server

Simple: open the terminal after starting roscore and type:
```
$ rosrun communication TCP_Server_node
```


## Start the TCP_Client

Open the terminal after starting roscore and type:
```
$ rosrun communication TCP_Client_node SERVER_IP
```
e.g.:
```
$ rosrun communication TCP_Client_node 192.168.2.115
```

## Sending a MSG

For this purpose, the client and the server provide a service. The service is called **"com_send_msg "**. 
The msg type is **SendTCPRequest** it expects two int32's.

**int32 action** This can be -1:error, 0:approach, 1:approached.

**int32 tokenNumber**     

## Receive a MSG
The server and client automatically publish the received MSG in the topic **"com_recv_msg "**
with the msg type **SendTCPRequest**.
