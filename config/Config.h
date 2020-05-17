#ifndef _CONFIG_H
#define _CONFIG_H

#define JETBOTURL "10.232.25.1"

namespace CONFIG {
    namespace MOTORS {
        enum MotorID{
            MOTORLEFT,
            MOTORIGHT
        };
    }
    namespace TELEOP {
        enum {
            PORT = 8080,
            ADDR = 0x00000000
        };
    }
}

#endif