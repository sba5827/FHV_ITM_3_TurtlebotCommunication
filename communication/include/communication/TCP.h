#ifndef TCP_H
#define TCP_H

#define ACTION_FLAG 'A'
#define NUMBER_FLAG 'N'

#include <communication/SendTCP.h>

namespace communication
{
    typedef enum {error=-1, approach, approached} Action_t;
    typedef int32_t TokenNumber_t;

    SendTCP::Request convertMsg_ctm(char msg[15]){

        SendTCP::Request ret;
        ret.action = (int32_t)error;
        ret.tokenNumber = -1;
        
        for(size_t i = 0; i < 14; i++) // only to 14 because pos 15 can not be an Flag
        {
            if(msg[i] == ACTION_FLAG){
                ret.action = (Action_t)msg[i+1];
            }
            else if(msg[i] == NUMBER_FLAG)
            {
                ret.tokenNumber = (int32_t)msg[i+1];
            }
            
        }

        return ret;
        
    }

    void convertMsg_mtc(communication::SendTCP::Request msg, char* buffer){
        
        buffer[0] = (char)ACTION_FLAG;
        buffer[1] = (char)msg.action;

        buffer[2] = (char)NUMBER_FLAG;
        buffer[3] = (char)msg.tokenNumber;

    }

} // namespace communication


#endif //TCP_H