# KR16-submission-folder

To run this package you need to make a workspace folder in Ubuntu through -mkdir, and then inside it another folder called src

In this src folder download the packages from https://github.com/ros-industrial/kuka_experimental, change the kuka_kr16_support from this link into the one in this repository, and add the other files

Then go into Ubuntu terminal, and path to the workspace and type -catkin_make, and after that source the through source/devel/setup.bash.

To run in the laberatory you need to start ROS core in one terminal window with -roscore

In another you need to run start_robot.launch with -roslaunch kuka_kr16_support start_robot.launch sim:=false (if sim:=true then you are not connected to the robot is sim:=false then you are connected)

To run the code open up another terminal window and run the code teleop.py
