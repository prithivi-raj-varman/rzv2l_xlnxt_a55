#ifndef CANCOMM_H
#define CANCOMM_H

/* C++ Standard Library Headers */
#include <iostream>
#include <string>
#include <cstring>      // for memset, memcpy
#include <cstdlib>      // for exit, malloc
#include <cstdio>       // for printf if needed
#include <cerrno>       // for errno
#include <thread>       // C++ threads (preferred over pthreads)
#include <atomic>
#include <chrono>

/* POSIX / UNIX System Headers */
#include <unistd.h>     // read, write, close, sleep
#include <fcntl.h>      // fcntl for non-blocking mode

/* Socket & Network Headers */
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>

/* Linux CAN Bus Headers */
#include <linux/can.h>
#include <linux/can/raw.h>

/* Optional (only if needed) */
#include <sys/select.h> // select()
#include <pthread.h>    // only if not using std::thread


/* Can Interfaces*/
#define CAN_INTERFACE "can0"

/*Can Node shifter*/
#define NODE_SHIFTER 6

/* Class declaration */
class CanComm {
    public:
        char* __can_iface, *__bitrate;
        int socketFD= -1;

        struct Command{
            unsigned char nodeID, command;
            unsigned long data;
        };

        struct can_frame _frame;
        struct ifreq ifr;
        struct sockaddr_can addr;

        CanComm();
        CanComm(const char* canIface, const char* bitRate, bool enableCAN);
        int init();
        //bool SendCanMessage(uint8_t nodeID, const uint8_t* data, uint8_t len);
        bool SendCanMessage(struct Command& cmd);
        bool ReceiveCanMessage();
        void handleReceivedMessage();
        ~CanComm();
    
    private:
        bool enableCan, isSending;
        unsigned char bigEndianArray[4];
};

#endif // CANCOMM_H