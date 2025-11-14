# UAV Obstacle Avoidance

> [!IMPORTANT]
Instructions on how to use the repository:

### Step 1:

Check if **ROS2** is installed in the system.
If not, install **ROS2** and its dependencies.

Install **MAVProxy** if not previously installed.

Libraries to be installed: **OpenCV, pyrealsense2, liberalsense2, GStreamer**

---

### Step 2:

Download the repository, unzip the folder and build the project.

`> cd ros2_workspace/ #open ros2_workspace folder and open terminal from that folder`\
`> colcon build`\
`> source install/setup.bash`

---

### Step 3:

To check if all the nodes are working properly, connect the UAV to power source and attach an HDMI to the NVIDIA onboard and let the onboard NVIDIA to turn on.\
Once the system is turned on, open a terminal and run below command.

`> mavproxy.py --master=/dev/ttyUSB0 --baudrate 921600 --out=udp:127.0.0.1:14550`

Output will be something like this:

![MAVLink connection successfull image](https://github.com/karanshah743/UAV_Obstacle_Avoidance/blob/main/images/Screenshot%20from%202025-09-29%2011-19-33.png)

This shows communication of NVIDIA with Pixhawk is live.

---

Next is to check publisher and subscriber node is working good.\
For that, keep running the MAVProxy command mentioned above ina a terminal and open two new separate terminal.\
Enter the commands below in each respectively.

`> ros2 launch realsense_object obd_avoid_launch.py`

and

`> ros2 launch realsense_object sub_launch.py`
