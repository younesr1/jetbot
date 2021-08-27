#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

struct GamepadEvent
{
    uint32_t time;
    int16_t data;
    uint8_t type;
    uint8_t id;
};

int main()
{
    auto fd = open("/dev/input/js0", O_RDONLY);
    while (fd == -1)
    {
        std::cout << "Unable to find PS4 controller. Trying again in 2 seconds" << std::endl;
        std::this_thread::sleep_for(2s);
        fd = open("/dev/input/js0", O_RDONLY);
    }

    while (true)
    {
        GamepadEvent event;
        if (read(fd, &event, sizeof(event)) != sizeof(event))
        {
            std::cout << "Read Failure" << std::endl;
        }
        else
        {
            std::cout << "{type " << (int)event.type << ", id " << (int)event.id << ", data " << (int)event.data << "}" << std::endl;
        }
    }
}