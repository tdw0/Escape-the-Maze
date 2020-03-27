# Escape-the-Maze
A reactive robot behaviour, programmed in Python using ROS (the Robot Operating System), that enables a robot to find a goal in a maze, utilising the robot’s sensors (LIDAR, Odometry and camera).

Launching the simulator:
```console
tom@desktop:~$ roslaunch uol_turtlebot_simulator simple.launch
```

Launching the RViz simulator:
```console
tom@desktop:~$ roslaunch uol_turtlebot_simulator turtlebot-rviz.launch
```

Launching the mazes to be traversed:
```console
tom@desktop:~$ roslaunch uol_turtlebot_simulator maze1.launch
tom@desktop:~$ roslaunch uol_turtlebot_simulator maze2.launch
tom@desktop:~$ roslaunch uol_turtlebot_simulator maze3.launch
```

