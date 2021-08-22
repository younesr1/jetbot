#pragma once
#include <array>

namespace IO
{
    /* A oop driver for a dualshock-ps4 controller */
    class Gamepad
    {
    private:
        using JoystickReading = std::array<int16_t, 2>;
        using TriggerReading = double;
        using ButtonReading = bool;

        struct GamepadState
        {
            JoystickReading left_js = {0, 0};
            JoystickReading right_js = {0, 0};
            TriggerReading right_trigger = 0;
            TriggerReading left_trigger = 0;
            ButtonReading left_bumper = false;
            ButtonReading right_bumper = false;
            ButtonReading triangle_button = false;
            ButtonReading square_button = false;
            ButtonReading x_button = false;
            ButtonReading circle_button = false;
        };

        struct GamepadEvent
        {
            uint32_t time;
            int16_t data;
            uint8_t type;
            uint8_t id;
        };

        GamepadState m_last_reading;

        std::string m_path;

        int32_t m_fd;

        /* Updates m_last_reading value */
        void Update(const GamepadEvent &event);

    public:
        Gamepad(const std::string &path = "/dev/input/js0");
        ~Gamepad();

        /**
    * @brief Attempts to connect the gamepad
    * @return success
    */
        bool Connect();

        /**
    * @brief Updates internal data structure
    * @return success
    */
        bool Update();

        /**
    * @brief returns copy of internal gamepad state
    * @return State
    */
        GamepadState Read() const;
    };
}