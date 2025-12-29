#include <iostream>

#include "CanComm.h"

/* Global variable */
extern int sockfd;
extern bool response;

CanComm::Command cmd_object = {0,0,0};

CanComm::CanComm() : CanComm(CAN_INTERFACE, "1000000", true) {
    // Default constructor delegates to parameterized constructor
}

CanComm::CanComm(const char* canIface, const char* bitRate, bool enableCAN) {
    __can_iface = strdup(canIface);
    __bitrate = strdup(bitRate);
    enableCan = enableCAN;
}

int CanComm::init() {
    /* Create CAN Socket */
    if ((socketFD = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cerr << "Error while opening socket: " << strerror(errno) << std::endl;
        return -1;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;

    strcpy(ifr.ifr_name, __can_iface);
    if (ioctl(socketFD, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    /* Bind the socket to CAN interface */
    if (bind(socketFD, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind: " << strerror(errno) << std::endl;
        return -1;
    }

    /* Set socket to non-blocking mode */
    int flags = fcntl(socketFD, F_GETFL, 0);
    if (flags < 0 || fcntl(socketFD, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("fcntl");
        close(socketFD);
        return -1;
    }
    sockfd = socketFD; // Assign to global variable
    return 0; // Placeholder return
}

bool CanComm::SendCanMessage(struct Command& cmd) {
    isSending = true;
    _frame.can_id = (cmd.nodeID << NODE_SHIFTER) | cmd.command;
    _frame.can_dlc = 8;
    // std::cout << "Sending CAN Message - ID: " << std::hex << _frame.can_id << std::dec << std::endl; //<< ", Data: " << cmd.data 

    // for (int i = 0; i < _frame.can_dlc; ++i) {
    //     _frame.data[i] = (cmd.data >> (i * 8)) & 0xff;
    //     std::cout << " " << std::to_string(_frame.data[i]);
    // }
    // std::cout << std::endl;
    std::cout << "Sending CAN Message - ID: " << std::hex << _frame.can_id << std::dec << std::endl;
    std::cout << "Data: ";
    for (int i = 0; i < _frame.can_dlc; ++i) {
        _frame.data[i] = (cmd.data >> (i * 8)) & 0xFF;
        std::cout << "0x" << std::hex << std::uppercase << (int)_frame.data[i] << " ";
    }
    std::cout << std::dec << std::endl;

    /* Check if write is success */
    if (write(socketFD, &_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        std::cerr << "Error in CAN message send: " << strerror(errno) << std::endl;
        isSending = false;
        return false;
    }
    return true;
}

// bool CanComm::SendCanMessage(uint8_t nodeID, const uint8_t* data, uint8_t len)
// {
//     struct can_frame frame = {};
//     frame.can_id  = nodeID;
//     frame.can_dlc = len;

//     memcpy(frame.data, data, len);

//     if (write(socketFD, &frame, sizeof(frame)) != sizeof(frame)) {
//         std::cerr << "Error sending CAN message" << std::endl;
//         return false;
//     }
//     return true;
// }

bool CanComm::ReceiveCanMessage() {
    struct can_frame frame;
    int nbytes; 

    while (1) {
        fd_set readfds;
        struct timeval timeout;

        FD_ZERO(&readfds);
        FD_SET(socketFD, &readfds);

        timeout.tv_sec = 1;  // 1 second timeout
        timeout.tv_usec = 0;

        int ret = select(socketFD + 1, &readfds, NULL, NULL, &timeout);
        if (ret < 0) {
            std::cerr << "Select error: " << strerror(errno) << std::endl;
            return false;
        } 

        if (FD_ISSET(socketFD, &readfds)) {
            nbytes = read(socketFD, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return false;
                } else {
                    std::cerr << "Error reading CAN message: " << strerror(errno) << std::endl;
                    return false;
                }
            }        

            uint32_t arbitrationId = frame.can_id & CAN_EFF_MASK;

            for (int i = 0; i < frame.can_dlc; ++i) {
                std::string dataReceived = std::to_string(frame.data[i]);
                bigEndianArray[i] = frame.data[3-i];
            }

            uint32_t nodeId = (arbitrationId >> NODE_SHIFTER);
            uint8_t commandId = arbitrationId & 0x3F;

            cmd_object.command = commandId;
            cmd_object.nodeID = nodeId;
            cmd_object.data = (unsigned long)frame.data;

            CanComm::handleReceivedMessage();
            return true;
        }    
        return false;
    }
}

void CanComm::handleReceivedMessage() {
    switch (cmd_object.command) {
        case 0x01:
            std::cout << "Received Command 0x01 from Node " << static_cast<int>(cmd_object.nodeID) << " with Data: " << cmd_object.data << std::endl;
            break;
        case 0x02:
            std::cout << "Received Command 0x02 from Node " << static_cast<int>(cmd_object.nodeID) << " with Data: " << cmd_object.data << std::endl;
            break;
        default:
            std::cout << "Received Unknown Command " << static_cast<int>(cmd_object.command) << " from Node " << static_cast<int>(cmd_object.nodeID) << " with Data: " << cmd_object.data << std::endl;
            break;
    }
}

CanComm::~CanComm() {
    if (socketFD != -1) {
        close(socketFD);
    }

    if (__can_iface) {
        free(__can_iface);
    }
    
    if (__bitrate) {
        free(__bitrate);
    }
}