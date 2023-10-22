# Assignment 1 - Reactive ROS Robot

## Description

This project involves creating a 2D robot that reactively follows a wall. The robot is round with differential locomotion. The environment is a “?” question mark shaped wall with a straight line below and an imperfect circle above. The robot starts at a random position inside the roundish part and wanders until it finds a wall. It then follows the wall until it reaches the rectangular sharp tip, which is the end position.

## Technologies Used

- ROS
- Python
- Flatland Simulator
- Rviz

## Installation and Dependencies

- Install ROS-2 Humble following the instructions [here](hhttps://docs.ros.org/en/humble/Installation.html)

## Usage

- Create a new workspace using the command `mkdir -p ~/ros2_ws/src`
- Navigate to the workspace using the command `cd ~/ros2_ws`
- Build the workspace using the command `colcon build`
- Source the workspace using the command `source /opt/ros/humble/setup.bash`
- Copy the contents of the repository into the src folder of the workspace
- Open a terminal in the folder you just copied the contents to
- Check for any missing dependencies using the command `rosdep install -i --from-path src --rosdistro humble -y`
- Build the workspace using the command `colcon build`
- Source the workspace using the command `source install/setup.bash`
- Run the launch file using the command `ros2 launch turtle turtle.launch.py`

## Additional Information

- For insights on the information about the last program odometry (path, distance, time), please run plotting.py file. Data will be saved in _data_info.csv_ .
