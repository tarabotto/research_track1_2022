Assignment 2
================================

This is the Luca Tarabotto's second assignment for the research track 1 course.
It's a ROS package that implements an action client and its interface in order to communicate with the given [action server](https://github.com/CarmineD8/assignment_2_2022).
The playground consists in a simulation of a 3D Robot able to move in a limited area, the user can digit the destination coordinates in a console interface and cancel or confirm the run after sending the input.

Requirements
-------------

- ROS [(Installation Guide)](http://wiki.ros.org/noetic/Installation/Ubuntu)
- C++ [(Installation Guide)](https://www.codespeedy.com/how-to-install-cpp-on-linux/)
- Xterm [(Installation Guide)](https://zoomadmin.com/HowToInstall/UbuntuPackage/xterm)
- ActionLib
  - Install in Console with:
```bash
$ apt-get install ros-noetic-actionlib
```
- ActionLib msgs
  - Install in Console with:
```bash
$ apt-get install ros-noetic-actionlib-msgs
```



Installing and running
----------------------

- Download the package from the repository
- Save it in your workspace
- Change Directory in the package path
  - Build your package with:
```bash
$ catkin_make
```
- Launch package with:
```bash
$ roslauch assignment2_tarabotto assignment.launch
```

Action Client Pseudocode
------------------------ 

Initialize global variables `p_x`, `p_y`, `vel_x`, `vel_z`, `goal_x`, `goal_y`, `cancel_flag`, `goal_flag`

Initialize **send_coord** service function
  set `goal_x` and `goal_y` at the value of the request part in the `Send` message
  set the value of the response part in the `Send` message to `goal_x` and `goal_y` 
  set `goal_flag` to **1**

Initialize **cancel_goal** service function
  set `cancel_flag` to **1**
  set the flag in the response part in the `Cancel` message to **1** 

Initialize **odomCallback** CallBack function
  set `p_x`, `p_y`, `vel_x`, `vel_z` at the corresponding values of the message received in the topic `/odom`

**Start of the Main Function**

Initialize the Node `action_client`  
Initialize Node Handle `nh`  

Create the Action Client `ac` that communicates with the Action server `/reaching_goal`
Wait for server

Create Publisher `pub` that publish in the topic `custom_topic`
Create Subrscriber `sub2` that subscribe to the topic `/odom` and calls **odomCallback** CallBack function

Create Service `send_service` that sends coordinates by calling the function `send_coord` that compiles the `Send` message 
Create Service `cancel_service` that cancel the run by calling the function `cancel_goal` that compiles the `Cancel` message

Initialize `goal` message
Initialize `odometry` message

While the node is running

  if `cancel_flag` is **1**
	communicate the Server to Cancel Goal
	reset `cancel_flag`

  if `goal_flag` is **1**
	compile `goal` message fields with `goal_x` and `goal_y`
	communicate the Server to Set Goal
	reset `goal_flag`

  refreshing the topics
  compile `odometry` message fields with `p_x`, `p_y`, `vel_x`, `vel_z`
  `pub` publishes the `odometry` message


Interface Pseudocode
------------------------

Initialize global variable `state`

Initialize **goalCallback** CallBack function
  set `state` at the corresponding values of the message received in the topic `/reaching_goal/result`


Initialize **insert_x** function
  print a request to the user to digit the X coordinate of the goal
  save the input in the local variable `x_insert` 
  return `x_insert`

Initialize **insert_y** function
  print a request to the user to digit the Y coordinate of the goal
  save the input in the local variable `y_insert` 
  return `y_insert`


Initialize **abortion_handler** boolean function
  print a request to the user to digit '*Y*' or '*N*' in order to cancel or not the run
  save the input in the local variable `response` 
  refreshing the topics

if the `response` is positive **and** the robot `state` is "*not arrived*"
    print user feedback
    return **0**

else if the `response` is negative **and** the robot `state` is "*not arrived*"
    print user feedback
	return **1**

else if the robot `state` is "*arrived*"
	print user feedback
	return **1**

else the input is not what expected
	print user feedback
	recall **abortion_handler** boolean function and save the output in the local bool variable `retry`
	return `retry`


Initialize **show_progress** function whith (`i`, `message`) as inputs
  if `i` is **0**
	print the `message`
	add 1 to `i`

  else if `i` is between 1 and 8
	print a dot
	add 1 to `i`

  else if `i` is between 8 and 16
	go back with the cursor and print a space replacing the last dot
	add 1 to `i`

  else 
	reset `i` to **1**


**Start of the Main Function**

Initialize the Node `interface`
Initialize Node Handle `nh`

Initialize local variable `x`
Initialize local variable `y`
Initialize local variable `interface_flag`
Initialize local variable `cancel_feedback`

Create Subrscriber `sub` that subscribe to the topic `/reaching_goal/result` and calls **goalCallback** CallBack function

Create service client `send_client` that calls `/ac_send` service
Create service client `cancel_client` that calls `/ac_cancel` service

Initialize `send_srv` service message
Initialize `cancel_srv` service message

While the node is running
  reset `state`
  save the output of **insert_x** function in `x`
  save the output of **insert_y** function in `y` 
  compile `send_srv` message fields with `x` and `y`
  `send_client` calls `/ac_send` service using the `send_srv` message
  print user feedback

  save the output of **abortion_handler** function in `interface_flag`
 
  if `interface_flag` is **1**
	initialize local variable `i`
	set refresh frequency
	while the robot `state` is not "*arrived*"
	  refresh the topics 
	  if the robot `state` is "*arrived*"
	    break the while loop
	  save the output of **show_progress**(`i`,"*The Robot is moving*") function in local variable `i`
	  apply refresh frequency delay
	print run finished feedback

  else if `interface_flag` is **0**
	`cancel_client` calls `/ac_cancel` service using the `cancel_srv` message
	save the response part of `cancel_srv` message in `cancel_feedback` local variable
	while `cancel_feedback` is empty
	  print error feedback
	  `cancel_client` calls `/ac_cancel` service using the `cancel_srv` message
	  save the response part of `cancel_srv` message in `cancel_feedback` local variable
	initialize local variable `i`
	set refresh frequency
	while the robot `state` is not "*cancelled*"
	  refresh the topics
	  if the robot `state` is "*cancelled*"
	    break the while loop
	  save the output of **show_progress**(`i`,"*Cancelling goal*") function in local variable `i`
	  apply refresh frequency delay
	print run cancelled feedback
