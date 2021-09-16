# Research Track I - final assignment
Roberta Reho - 5075214

INDEX
---------------------
 * Contents of the package
 * Computational graph of the system
 * Roslaunch
 * Installation and running
 * Robot behaviors implemented
 * Software architecture 
 * System's limitations and possible improvements


## Contents of the package
The pakage contains new nodes that implement some new behaviours, other than the ones already seen in previous exercises (bug_m.py, go_to_point.py, wall_follow_service.py).

### controller.py
Is the main node of the architecture: it publishes to the topics move_base/goal and move_base/cancel to set and cancel target coordinates for the move_base algorithm. Moreover, it publishes to the topic /cmd_vel to set the robot's velocity.
The node manages the different behaviours through the services displayed below. 
At the beginning of the routine it calls the user_interface service in order to acquire the commands from the user. Depending on the command, the appropriate services and will be activated to perform the right behaviour.
The node is subscribed to the topic /move_base/result that publishes a message every time a target has been reached with the move_base algorithm.

### user_interface.py
Is a service that allows the user to input the command matching the desired behaviour:
1. Move randomly to one of the positions: (-4,3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1) 
2. Manually inser one of the 6 positions among the list above;
3. Start following the external walls;
4. Stop in the last position;
5. Change algorithm;

In case the chosen behaviour is number 2, the inteface will display a further message, allowing to insert one of the available coordinates. A while cycle is implemented to verify the validity of the input.
The screen output of the node is printed in a new window, opened though the launch file.

### state_publisher.py
Is a separate node that prints some relevant information on the main terminal.
It is subscribed to the topics /scan and /odom in order to receive updated datas reguarding the position of the robot and the laser distances.
At every cycle the nodes prints:
* the content of the parameter "state" among 'Go to point', 'wall following', 'target reached','stopped'
* the target to be reached in case of mode 1 or 2
* x, y, z coordinates of the robot position
* the distances of the obstacles in the main regions "right", "fright", "front", "fleft", "left"
* the boolean parameter "target_reached"

The informations are printed every 3 seconds to prevent the terminal from being crowded.
These information will be displayed together with all the outputs of the other nodes except for the user interface.

### random_target_service.py
The service server is called by the client controller.py in case the input command is "1".
The server is sent an empty message and generates random coordinates among the available ones. Then updates the corresponding parameters so that the coordinates are available to multiple nodes.

### bug_m.py
The bug algorithm node is mainly the same as the provided one, with a few modifications:
* The initial state is set on "target reached" instead of "go to point" (2 in place of 0);
* A new service allows the algorithm to be switched on and off from an external client, in this case, implemented in the controller node;
* The "change_state" function is now able to access the global parameter "target_reached" and switch its state, as well as the parameter "state";
* A 40 seconds timer has been implemented to avoid Bug0 algorithm to get stuck trying to reach and unreachable target. The timer starts as soon as the robot is reaching for its target and, if it expires before the goal is reached, it returns the state "target_reached" anyway.

### wall_follower.py and go_to_point.py
The modes "wall_follower" and "go_to _point" are the same as the ones provided originally.


## Computational graph of the system
![alt text](https://github.com/RobReho/RT_assignment2/blob/main/rosgraph.png)

## Roslaunch 
The application can be run with a single launch file "final.launch". It includes "simulation_gmapping.launch" and "move_base.launch" from the same folder and runs all the nodes and services listed above.
Moreover, a series of parameters are defined:
* mode: int variable from 1 to 5, indicates the current robot mode among the 5 available;
* state: int variable from 0 to 3, respectively "go to point","wall follower","target reached","stopped";
* des_pos_x / des_pos_y: target coordinates set by the random target service or the user interface;
* target_reached: bolean variable, states whether the target has been reached or not;
These parameters are a publish to blackboard-like communication paradigm between nodes.

## Installation and running
The package final_assignment requires the packages [move_base](http://wiki.ros.org/move_base) and [gmapping](http://wiki.ros.org/gmapping) in order to work proprely.
Moreover, the robot URDF is taken from the package [robot_description](https://github.com/CarmineD8/robot_description)
The package also uses Rviz and Gazebo.
The application can be run by launching the final.lounch file in the launch folder

```sh
roslaunch final_assignment final.launch
```


## Robot behaviors implemented
The robot can behave differently according to the user input:
* Mode 1 - the robot is given a random target among (-4,3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1), then it starts reaching it with the active algorithm.
* Mode 2 - the user as asked to manually insert a target among the same listed above. Again, after checking whether the input is valid, the robot goes towords the target using the active algorithm.
* Mode 3 - If the last target set has been reached, the robot starts searching for a wall and follows it.
* Mode 4 - The robot stops in the current position.
* Mode 5 - Is a switcher between Move base and Bug0 algorithms.

After a mode is inserted the user interface is immediately available to receive a new command.
In case of both modes 1 and 2 the robots stops once the target position has been reached and waits for a new command. If the target has not been reached yet it is still possible to call for behaviour 1 or 2 and insert a new target that will be taken charge of right away.
If the user wants to change from modes 1 or 2 to 3 and 5, the robot will wait until after the current target has been reached. On the other hand, it is possible to immediately switch from mode 3 to any other. 

## Software architecture 
State, mode and target coordinates are defined as parameters since multiple nodes need access to it depending on the behaviour to be executed or the algorithm used. For this reason, a publish to blackboard communication paradigm seemed the most appropriate.
The user interface node is called on a new terminal through

```
launch-prefix="terminator -x python"
```
in the launch file. This choice was made to have a tidy terminal completely dedicated to acquiring commands, separated from the messages printed in the main terminal.
State publisher has been implemented as a separate node printing periodically on the main terminal along some other messages produced by the rest of the nodes (except for the user interface). The node takes informations from global variables and parameters and its modular nature allows it to be easily disabled for debugging purposes.

## System's limitations and possible improvements
For unknown reasons the topic /move_base/result takes up to 20 seconds before publishing the result message once the target has been reached with move_base algorithm.
A further improvement for this architecture could be the better management of global parameters that appear redundant in certain cases.






