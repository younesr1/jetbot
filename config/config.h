#pragma once
#include <cstdint>
#include <chrono>
#include <string>

namespace CONFIG {
    namespace MOTORS {
        constexpr uint8_t DEADZONE = 5;
        enum {
            MOTORLEFT = 1,
            MOTORRIGHT = 2,
        };
    }
    namespace CONTROLLER {
        constexpr uint8_t LEFTJSX = 0;
        constexpr uint8_t LEFTJSY = 1;
        constexpr uint8_t JS_EVENT_AXIS = 0x02;
        constexpr uint8_t JS_EVENT_INIT = 0x80;
        constexpr uint8_t L2 = 3;
        constexpr uint8_t R2 = 4;
        constexpr uint16_t MAX_JS = 32767;
    }
    namespace MOTORCONTROLLER {
        constexpr auto SLEEP = std::chrono::microseconds(500);
    }
    namespace CAMERA {
        constexpr uint16_t WIDTH = 1280;
        constexpr uint16_t HEIGHT = 720;
        constexpr uint8_t FPS = 10;
        constexpr uint8_t FLIP = 4; // vertical flip
        constexpr auto RECORDTIME = std::chrono::seconds(20);
    }
    namespace SLAM {
        constexpr uint16_t MAC_FEATURES = 3000;
        constexpr uint8_t MINDIST = 3;
        constexpr uint8_t KNNDIMENSION = 2;
        constexpr uint8_t KPDIAMETER = 20;
        constexpr float REJECTIONREGION = 0.01F;
        constexpr float FILTERINGTHRESHOLD = 0.02F;
        constexpr float RATIO = 0.60f;
        constexpr uint8_t FEATURECIRCLESIZE  = 2;
    }
}