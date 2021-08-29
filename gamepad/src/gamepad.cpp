#include "gamepad/gamepad.hpp"
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace IO
{
    Gamepad::Gamepad(const std::string &path) : m_fd(-1), m_path(path)
    {
    }

    bool Gamepad::Connect()
    {
        m_fd = open(m_path.c_str(), O_RDONLY | O_NONBLOCK);
        return m_fd != -1;
    }

    Gamepad::GamepadState Gamepad::Read() const
    {
        return m_last_reading;
    }

    bool Gamepad::Update()
    {
        GamepadEvent event;
        bool success = read(m_fd, &event, sizeof(event)) == sizeof(event);
        if (success)
        {
            Update(event);
        }
        return success;
    }

    void Gamepad::Update(const Gamepad::GamepadEvent &event)
    {
        // jetson and laptop have different button mappings for ps4 controller
        constexpr auto AXIS = 0x02; // joystick and trigger
        constexpr auto BUTTON = 0x01;
#ifdef __ARM_ARCH_8A
        constexpr auto RIGHT_TRIGGER = 0x04, LEFT_TRIGGER = 0x03;
        constexpr auto LEFT_JS_X = 0x00, LEFT_JS_Y = 0x01;
        constexpr auto RIGHT_JS_X = 0x02, RIGHT_JS_Y = 0x05;
        constexpr auto X_BUTTON = 0x1, CIRCLE_BUTTON = 0x02, TRIANGLE_BUTTON = 0x03, SQUARE_BUTTON = 0x00;
        constexpr auto RIGHT_BUMPER = 0x05, LEFT_BUMPER = 0x04;
#endif
#ifdef __x86_64__
        constexpr auto RIGHT_TRIGGER = 0x05, LEFT_TRIGGER = 0x02;
        constexpr auto LEFT_JS_X = 0x00, LEFT_JS_Y = 0x01;
        constexpr auto RIGHT_JS_X = 0x03, RIGHT_JS_Y = 0x04;
        constexpr auto X_BUTTON = 0x0, CIRCLE_BUTTON = 0x01, TRIANGLE_BUTTON = 0x02, SQUARE_BUTTON = 0x03;
        constexpr auto RIGHT_BUMPER = 0x05, LEFT_BUMPER = 0x04;
#endif
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
                m_last_reading.left_js[0] = event.data;
            }
            else if (event.id == LEFT_JS_Y)
            {
                m_last_reading.left_js[1] = event.data;
            }
            else if (event.id == RIGHT_JS_X)
            {
                m_last_reading.right_js[0] = event.data;
            }
            else if (event.id == RIGHT_JS_Y)
            {
                m_last_reading.right_js[1] = event.data;
            }
        }
        else if (event.type == BUTTON)
        {
            if (event.id == X_BUTTON)
            {
                m_last_reading.x_button = event.data;
            }
            else if (event.id == CIRCLE_BUTTON)
            {
                m_last_reading.circle_button = event.data;
            }
            else if (event.id == SQUARE_BUTTON)
            {
                m_last_reading.square_button = event.data;
            }
            else if (event.id == TRIANGLE_BUTTON)
            {
                m_last_reading.triangle_button = event.data;
            }
            else if (event.id == RIGHT_BUMPER)
            {
                m_last_reading.right_bumper = event.data;
            }
            else if (event.id == LEFT_BUMPER)
            {
                m_last_reading.left_bumper = event.data;
            }
        }
    }

    Gamepad::~Gamepad()
    {
        close(m_fd);
    }
}
