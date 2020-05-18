#include "Server.h"

Server::Server() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        perror("socket creation failed"); 
        exit(EXIT_FAILURE);
    }
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr));
    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(CONFIG::TELEOP::PORT); 
    // Bind the socket with the server address 
    if(bind(sockfd, (sockaddr *) &servaddr, sizeof(servaddr)) < 0 ) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    }
}

void Server::run() {
    std::thread workerThread(serve);
    std::cout << "Server thread initiated." << std::endl;
}

void Server::serve() {
    uint8_t payload[2] = {0};
    while (true) {
        recvfrom(sockfd, payload, sizeof(payload), MSG_WAITALL, (sockaddr *) &cliaddr, &cliAddrLen);
        // now do stuff with payload[0] and payload[1]
        std::this_thread::sleep_for(1s);
    }
    
}

Server::~Server() {
    close(sockfd);
}