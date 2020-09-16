# Jetbot Localizer & Mapper

The goal of this personal project was to learn more about openCV and a fundamental computer vision problems in Robotics, SLAM. Loosely, the Simultaneous Localization and Mapping (SLAM) problem is one where an agent looks to create a map of its environment while estimating its pose within that environment. Specifically, Monocular SLAM relies solely on the cameraSo far, I have been able to locate discernible features in video frames and track their motion across frames. At this point, I have yet to start working on pose estimation of the rover.

# Summary
I have written code that identifies, filters and tracks features across frames on the Nvidia Jetbot. I put the Jetbot on my dashboard and drove around. Green circles represent the features the algorithm is tracking. Blue lines connect the  location of a feature in the previous frame to its location in the current frame. Consequently, very short blue lines indicate accurate feature tracking, while long cross-frame blue lines indicate noise. Here is the output: **ADD GIF**

# Assembly

For this project, I assembled the Jetbot, Nvidia's hobbyist robotics platform based on the Jetson Nano. Nvidia conveniently shares the assembly instructions for the robot along with the CAD for the chassis. I 3D printed the chassis through an online service and sourced most of the parts from amazon. The Jetbot consists of a rechargeable battery, a motor driver, 2 motors, and a 160° FOV Raspberry Pi Camera.
**ADD IMAGE**

# Control

After assembling the Jetbot, it was time to make it move! I wired the motor driver to the I2C pins on the Jetson Nano and wrote some code to control the speed of the motors. Next, I wrote some code to take in input from a connected PS4 controller via Bluetooth and linked these two modules together. Finally, I wrote some code for the Pi camera to record video while the rover drives around.

# Vision
Next, I looked to implement cross-frame feature tracking. I was largely guided by a repo I found on Github, which implemented SLAM in Python. I only often referred to the openCV documentation and their detailed tutorials. Essentially, the feature extracting method was the following:
 1. Identify Locators of Features Worth Tracking
 2. Generate Descriptors from Locators
 3. Compare Descriptors Across Frames with a Brute Force Matcher
 4. Use the K Nearest Neighbours Method to Match Descriptors
 5. Filter Features with the ratio test 

# Running the Code
Initially, the Jetbot was meant to be aroom-mapping robot. However, the rooms in my house didn't have very many features worth tracking. I noticed that the feature tracking algorithm worked best with sharp corners and tree foliage. For this reason, I decided to mount the Jetbot on my dashboard and go for a drive around my hometown.

Green circles represent the features the algorithm is tracking. Blue lines connect the  location of a feature in the previous frame to its location in the current frame. Consequently, very short blue lines indicate accurate feature tracking, while long cross-frame blue lines indicate noise. Below is a GIF of the output. **ADD GIF**

# TODO

This is an ongoing project. I have yet to implement pose estimation of the rover. The ultimate goal of the project is to create a point cloud representation of the Jetbot's surroundings and generate a rendering of its pose as it traverses the environment.

# Credits

Link to Jetbot assembly: [link](https://github.com/NVIDIA-AI-IOT/jetbot/wiki)
Link to Python SLAM: [link](https://github.com/geohot/twitchslam)

