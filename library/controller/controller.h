#pragma once
#include "boost/optional.hpp"
#include "config.h"
#include <cstdint>

class controller {
    private:
    int32_t _fd;
    uint8_t _pf;
    uint8_t _lr;
    uint8_t _rr;
    struct motorSpeeds {
        int motorLeftSpeed;
        int motorRightSpeed;
    };
    struct {
        uint8_t type, number;
        int16_t value;
        uint32_t time;
    }_event;
    bool sample();
    bool R2triggered();
    bool leftJStriggered();
    public:
    controller();
    ~controller();
    boost::optional<motorSpeeds> pollOnce();
};