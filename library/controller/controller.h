#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <cstdint>
// CONFIG FILE SOMEWHERE?
#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define R2 5
#define LEFTJSX 0
#define LEFTJSY 1
#define MAX_JS 32767
class controller {
    public:
    controller();
    ~controller();
    bool sample();
    void run();
    private:
    // file
    int32_t _fd;
    // power factor
    uint8_t _pf;
    // left ratio
    int8_t _lr;
    // right ratio
    int8_t _rr;
    struct {
        uint8_t type, number;
        int16_t value;
        uint32_t time;
    }_event;
    bool R2triggered();
    bool leftJStriggered();
};
#endif 