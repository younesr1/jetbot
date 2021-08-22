# JetBot Model for Gazebo Robotics Simulator

### Credit
This folder was taken from [here](https://github.com/dusty-nv/jetbot_ros/blob/master/gazebo/README.md)

### Installation

To setup the model, first install and run Gazebo once, and then run the following script:

```bash
$ cd jetbot/gazebo   # substitute where you have jetbot repo on your machine
$ ./install_jetbot_model.sh
```

The `jetbot` model will then be symbolically linked to `~/.gazebo/models` and can then be loaded with Gazebo.

> **note**:  to uninstall the model, run `rm ~/.gazebo/models/jetbot`
