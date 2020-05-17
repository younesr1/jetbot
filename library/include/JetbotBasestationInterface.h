#ifndef _JETBOT_BASE_STATION_INTERFACE_H
#define _JETBOT_BASE_STATION_INTERFACE_H
#include <iostream>
#include <curl/curl.h>
#include "config.h"
// when compiling, must link $ gcc test.c -lcurl bc external lib
class JetbotBasestationInterface {
    public:
    JetbotBasestationInterface();
    ~JetbotBasestationInterface() {
        curl_easy_cleanup(curl);
    }
    bool SendMotorData(uint8_t id, uint8_t val);
    private:
    CURL *curl;
    CURLcode ret;
};
#endif