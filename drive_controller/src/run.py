#!/usr/bin/env python
import time
from adafruit_motorkit import MotorKit
# younes todo create proper fw library and use it in both node and here
# younes todo this does not terminate well use ros::ok
# younes todo the node.py uses .throttle(speed) instead of .throttle = speed, fix that
# also the fucking motor driver beeps at no speed (torque aint strong enough)
def main():
    kit = MotorKit()
    while True:
        kit.motor1.throttle = 0.5
        time.sleep(0.5)
        kit.motor1.throttle = 0
        time.sleep(0.5)

        
if __name__ == '__main__':
    main()

