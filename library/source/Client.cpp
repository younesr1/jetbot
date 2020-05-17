#include "Client.h"

Client::Client() {
    sockfd = socket(AF_INET, SOCK_DGRAM,0);
    if(sockfd < 0) {
        std::perror("socket creation failed");
        std::exit(EXIT_FAILURE);
    }
    memset(&servaddr, 0, sizeof(servaddr));
    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(CONFIG::TELEOP::PORT); 
    servaddr.sin_addr.s_addr = CONFIG::TELEOP::ADDR;
}

void Client::SendMotorData(CONFIG::MOTORS::MotorID id, uint8_t val) {
    uint8_t payload[] = {id, val};
    sendto(sockfd, payload, sizeof(payload), MSG_CONFIRM, (sockaddr *) &servaddr, sizeof(servaddr));
}