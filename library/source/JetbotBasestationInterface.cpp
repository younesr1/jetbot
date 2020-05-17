#include "JetbotBasestationInterface.h"

JetbotBasestationInterface::JetbotBasestationInterface() {
    curl = curl_easy_init();
    if(!curl) {
        std::cout << "ERROR INITIALZING JETBOTBASESTATIONINTERFACE." << std::endl;
        // do a better job raising flag
    }
}

JetbotBasestationInterface::~JetbotBasestationInterface() {
    curl_easy_cleanup(curl);
}

bool JetbotBasestationInterface::SendMotorData(CONFIG::MOTORS::MotorID id, uint8_t val) {
    /*std::string field;
    curl_easy_setopt(curl, CURLOPT_URL, JETBOTURL);
    field = "motorID=" + std::to_string(id) + "&value=" + std::to_string(val);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, field);
    ret = curl_easy_perform(curl);
    curl_easy_reset(curl);
    return ret == CURLE_OK;*/
}