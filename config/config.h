#ifndef _CONFIG_H
#define _CONFIG_H

#define JETBOTURL "10.232.25.1"

namespace CONFIG {
    namespace MOTORS {
        enum MotorID : uint8_t{
            MOTORLEFT,
            MOTORIGHT
        };
    }
    namespace TELEOP {
        enum uint32_t{
            PORT = 8080,
            JETBOTADDR = 0x00000000,
            BASESTATIONADDR = 0x00000001
        };
    }
}

#endif