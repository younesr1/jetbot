#ifndef _CLIENT_H
#define _CLIENT_H
#include <iostream>
//#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include "config.h" 
#define MAXLINE 64
// when compiling, must link $ gcc test.c -lcurl bc external lib
class Client {
    public:
    ~Client();
    Client();
    void SendMotorData(CONFIG::MOTORS::MotorID id, uint8_t val);
    private:
    int sockfd;
    sockaddr_in servaddr;
};
#endif