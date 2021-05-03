#include "gamepad/gamepad.hpp"
#include <sys/stat.h>
#include <fcntl.h>
#include <filesystem>
#include <unistd.h>

namespace IO
{
    Gamepad::Gamepad() : m_fd(-1)
    {
        constexpr auto path = "/dev/input/js0";
        m_fd = open(path, O_RDONLY);
        if (m_fd == -1)
        {
            throw std::filesystem::filesystem_error("No gamepads detected", path, std::error_code(2, std::system_category()));
        }
    }

    Gamepad::JoystickReading Gamepad::ReadLeftJoystick() const
    {
        constexpr auto scaling_factor = 100.0 / std::numeric_limits<Gamepad::Resolution>::max();
        return {scaling_factor * m_last_reading.left_js_x, -1 * scaling_factor * m_last_reading.left_js_y};
    }

    Gamepad::JoystickReading Gamepad::ReadRightJoystick() const
    {
        constexpr auto scaling_factor = 100.0 / std::numeric_limits<Gamepad::Resolution>::max();
        return {scaling_factor * m_last_reading.right_js_x, -1 * scaling_factor * m_last_reading.right_js_y};
    }

    Gamepad::TriggerReading Gamepad::ReadLeftTrigger() const
    {
        constexpr auto scaling_factor = 100.0 / std::numeric_limits<Gamepad::Resolution>::max();
        constexpr auto offset = 100;
        return (scaling_factor * m_last_reading.left_trigger + offset) / 2;
    }

    Gamepad::TriggerReading Gamepad::ReadRightTrigger() const
    {
        constexpr auto scaling_factor = 100.0 / std::numeric_limits<Gamepad::Resolution>::max();
        constexpr auto offset = 100;
        return (scaling_factor * m_last_reading.right_trigger + offset) / 2;
    }

    bool Gamepad::Update()
    {
        GamepadEvent event;
        bool success = read(m_fd, &event, sizeof(event)) == sizeof(event);
        if (success)
            Update(event);
        return success;
    }

    void Gamepad::Update(const Gamepad::GamepadEvent &event)
    {
        constexpr auto AXIS = 0x02;
        constexpr auto RIGHT_TRIGGER = 0x05, LEFT_TRIGGER = 0x02;
        constexpr auto LEFT_JS_X = 0x00, LEFT_JS_Y = 0x01;
        constexpr auto RIGHT_JS_X = 0x03, RIGHT_JS_Y = 0x04;
        if (event.type == AXIS)
        {
            if (event.id == RIGHT_TRIGGER)
            {
                m_last_reading.right_trigger = event.data;
            }
            else if (event.id == LEFT_TRIGGER)
            {
                m_last_reading.left_trigger = event.data;
            }
            else if (event.id == LEFT_JS_X)
            {
                m_last_reading.left_js_x = event.data;
            }
            else if (event.id == LEFT_JS_Y)
            {
                m_last_reading.left_js_y = event.data;
            }
            else if (event.id == RIGHT_JS_X)
            {
                m_last_reading.right_js_x = event.data;
            }
            else if (event.id == RIGHT_JS_Y)
            {
                m_last_reading.right_js_y = event.data;
            }
        }
    }

    Gamepad::~Gamepad()
    {
        close(m_fd);
    }
}