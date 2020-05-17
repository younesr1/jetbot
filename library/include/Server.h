#ifndef _SERVER_H
#define _SERVER_H
#include <iostream>
#include "Config.h"
// when compiling, must link $ gcc test.c -lcurl bc external lib
class Server {
    public:
    Server();
    // spins a thread and waits for a request
    bool init();
    private:
};
#endif