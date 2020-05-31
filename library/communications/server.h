#ifndef _SERVER_H
#define _SERVER_H
#include <iostream>
#include <thread>
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include "config.h"
// when compiling, must link $ gcc test.c -lcurl bc external lib
class server {
    public:
    ~server();
    server();
    void run();
    private:
    void serve();
    int32_t sockfd;
    uint32_t cliAddrLen;
    sockaddr_in servaddr, cliaddr;
};
#endif