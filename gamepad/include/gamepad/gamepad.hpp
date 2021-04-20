#pragma once
#include <array>
#include <limits>
#include <mutex>

class Gamepad
{
private:
    using JoystickReading = std::array<double, 2>; // [-100, 100]
    using TriggerReading = double;                 // [0, 100]
    using ButtonReading = bool;
    using Resolution = int16_t;

    struct GamepadReading
    {
        Resolution left_js_x = 0, left_js_y = 0;
        Resolution right_js_x = 0, right_js_y = 0;
        Resolution right_trigger = std::numeric_limits<Resolution>::min();
        Resolution left_trigger = std::numeric_limits<Resolution>::min();
    };

    struct GamepadEvent {
        uint32_t time;
        Resolution data;
        uint8_t type;
        uint8_t id;
    };

    GamepadReading m_last_reading;

    int32_t m_fd;

    std::mutex m_mutex;

    /* Updates m_last_reading value */
    void Update(const GamepadEvent &event);

public:
    Gamepad();
    ~Gamepad();

    /**
    * @brief Updates internal data structure. Should run in its own thread
    * @return success
    */
    bool Update();

    /**
    * @brief Read the left joystick value from internal data structure
    * @param reading {x-axis, y-axis} [-100, 100]
    */
    JoystickReading ReadLeftJoystick();

    /**
    * @brief Read the right joystick from internal data structure
    * @param reading {x-axis, y-axis} [-100, 100]
    */
    JoystickReading ReadRightJoystick();

    /**
    * @brief Read the right trigger from internal data structure
    * @param reading 0 is unpressed, 100 is fully pressed
    */
    TriggerReading ReadRightTrigger();

    /**
    * @brief Read the left trigger from internal data structure
    * @param reading 0 is unpressed 100 is fully pressed
    */
    TriggerReading ReadLeftTrigger();
};