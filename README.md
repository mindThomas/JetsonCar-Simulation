# JetsonCar-Simulation
Simulation environment for the Jetson Car project using Gazebo and ROS

Clone this repository into an existing catkin workspace:
```bash
mkdir -p ~/JetsonProjects/src
cd ~/JetsonProjects/src
catkin_init_workspace
git clone https://github.com/mindThomas/JetsonCar-Simulation
```

Build the project with catkin build
```bash
cd ~/JetsonProjects
catkin build
source devel/setup.zsh
```

Published frames (TF's) can be viewed with (generates a PDF)
```bash
rosrun tf view_frames
```

Another way would be to convert the xacro file to urdf and then `urdf_to_graphiz`
```bash
xacro JetsonCar-Simulation/jetsoncar_description/urdf/jetsoncar.xacro >> jetsoncar.urdf
urdf_to_graphiz jetsoncar.urdf
evince jetsoncar.pdf
```



# Notes
`joint_state_publisher` is only for testing purposes. A node in a real system or Gazebo for simulation should provide the current actual joint angles by updating all tf's.

Both joint transmission (actuators) and joint controllers will have to be initialized: http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo
