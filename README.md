Assignment 1
================================

This is the Luca Tarabotto's first assignment for the research track 1 course.
Is based on the simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).
The robot must recognize some silver boxes, reach them, grab them and pairing them with the nearest free golden box.

Installing and running
----------------------

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/).

Once the dependencies are installed, simply run the `assignment1.py` in */research_track1_2022/robot-sim* path.


Pseudocode
---------

Initialize the empty list `pairs`  
Initialize `actual distance offset` to the initial `distance offset`  
Initialize the first `direction` to `right` (`1`)  
Initialize `try angle` to `zero`  

### Start the routine loop ###

**if** all boxes are paired  
&nbsp;&nbsp;&nbsp;&nbsp;close the program
	
	
**if** the robot is searching a *silver box*  
&nbsp;&nbsp;&nbsp;&nbsp;find `distance`,`angle` and `code` of the *silver boxes* in the field of view
	
**else**  
&nbsp;&nbsp;&nbsp;&nbsp;find `distance`,`angle` and `code` of the *golden boxes* in the field of view


**if** can't see any valid token  
&nbsp;&nbsp;&nbsp;&nbsp;**if** the robot has already looked all around  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;increase `actual distance offset`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;reset `try angle` to `zero`  
&nbsp;&nbsp;&nbsp;&nbsp;**else**  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;rotate and look around   

**else if** the `distance` is lower than the threshold **and** the robot is searching the silver tokens  
&nbsp;&nbsp;&nbsp;&nbsp;grab the token  
&nbsp;&nbsp;&nbsp;&nbsp;pass to golden token searching   
&nbsp;&nbsp;&nbsp;&nbsp;reset `try angle` to `zero`
	
**else if** the `distance`is lower than the threshold **and** the robot is searching the golden tokens  
&nbsp;&nbsp;&nbsp;&nbsp;release the silver box near the golden one  
&nbsp;&nbsp;&nbsp;&nbsp;pass to silver token searching	 
&nbsp;&nbsp;&nbsp;&nbsp;reset `try angle` to `zero`   
&nbsp;&nbsp;&nbsp;&nbsp;reset `actual distance offset`  

**else if** the `angle` is outside the grab range **and** the token is not in the `pairs`list  
&nbsp;&nbsp;&nbsp;&nbsp;correct the `angle`  
&nbsp;&nbsp;&nbsp;&nbsp;reset `try angle` to `zero`  
	
**else if** the `angle` is inside the grab range **and** the token is not in the `pairs`list  
&nbsp;&nbsp;&nbsp;&nbsp;move forward in the next token direction  
&nbsp;&nbsp;&nbsp;&nbsp;reset `try angle` to `zero`  





### Possible improvements ###

Since the robot has no knowledge of its place in the environment of the arena, it doesn't know the absolute position of the boxes.   
All it knows are the positions of the boxes in its field of view with respect to its placement in the arena.  
The code is developed in order to make the robot looks around on the left & right in search of the nearest box.  
These movements could be expensive in energetic prospectives. It could be smarter by adding a logic that gives the robot te perception   
of the boxes, even if them are behind it, by calculating and saving the distances and angles each time the robot moves, remembering the last time it saw the token.


