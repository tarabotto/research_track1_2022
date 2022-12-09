from __future__ import print_function

import time
from sr.robot import *

a_th = 2.0
""" float: Threshold for the control of the linear distance"""

d_th = 0.4
""" float: Threshold for the control of the orientation"""

silver = True
""" boolean: variable for letting the robot know if it has to look for a silver or for a golden marker"""

dist_offset=1.7
""" initial distance offset that determines if a token is close enough """

right_direction=1
""" Right=1, Left=-1 """

fov=50.0
""" field of view of the robot needed for the looking around rotation """


R = Robot()
""" instance of the class Robot"""


def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: speed (int): the speed of the wheels
	  seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the next silver token

    Returns:
	dist (float): distance of the closest silver token (-1 if no silver token is detected)
	rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
	code (int): unique code of the token
    """
    
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER:
            dist=token.dist
	    rot_y=token.rot_y
	    code=token.info.offset
	    
	    
    if dist==100:
	return -1, -1, -1
    else:
   	return dist, rot_y, code
   	
   	
def find_golden_token(actual_dist_offset):
    """
    Function to find the closest golden token
    Input:
    	actual_dist_offset (float): distance offset to determine if the golden token in front of the robot is close enough

    Returns:
	dist (float): distance of the closest golden token (-1 if no golden token is detected)
	rot_y (float): angle between the robot and the golden token (-1 if no golden token is detected)
	code (int): unique code of the token
	size (float): size of the token 
    """
    
    dist=actual_dist_offset
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            dist=token.dist
	    rot_y=token.rot_y
	    code=token.info.offset
	    size=token.info.size
	   
	    
    if dist==actual_dist_offset:
		return -1, -1, -1, -1
    else:
   		return dist, rot_y, code, size 	
   	
   	
   	
def turn_angle(angle,seconds):
	"""
    Function to rotate the robot by a specified angle in degrees, translate the desired rotation angle in speed needed by "turn" function
    
    Input:
    	angle(float): desired angle of rotation
    	seconds(float)	 
    """

	if seconds==0.0:
		speed=0.0
	else:	
		speed=angle/(3.0*seconds) #at speed=100, 360deg in 1.2sec
	
	if speed>100: #saturation
		speed=100
		seconds=1.2
		
	turn(speed,seconds)  
	
	
def look_around(try_angle, direction):	
	"""
	Function that makes the robot looking around to the left and to the right (like John Travolta meme :)) 
	it changes the angle in order to see if there is a token close enough in the field of view of the robot, 
	first rotation to the left, second to the right and so on
	Input:
		try_angle (float): actual angle of rotation where the robot is trying to find a valid token
	  	direction (int): the previous direction of the searching rotation 
	"""
	print("Can't see token near enough to me")			
	try_angle+=fov
	turn_angle(try_angle*direction,try_angle/135.0) 
	direction= -direction
	
	return try_angle,direction	


def increase_research(actual_dist_offset): 	
	"""
	Function that allows the robot to increase the distance offset if it can't see a token close enough, it turns of 180 degrees and will restart searching
	Input:
		try_angle (float): actual angle of rotation where the robot is trying to find a valid token
		actual_dist_offset (float): the actual distance offset used for the research of the token
	
	Returns:
	actual_dist_offset (float):	the increased distance of research of the robot
	"""
	turn_angle(180,1.5)
	actual_dist_offset+=0.4
	print("Mhhmhm... I'll search farther")
 			 
	return actual_dist_offset



						# MAIN CODE
												
												
											
						#VARIABLES DECLARATION 
 			
paired=[] #list that will be filled with paired golden & silver token codes

actual_dist_offset=dist_offset #Initialize the distance offset within the robot will search the next token

direction=right_direction #Initialize the initial direction where the robot will point to in the token-research "look_around" function

try_angle=0 #Initialize the Angle where the robot is pointing to in order to find a valid token

size=0.0 #inizialize the size of the token

												
						#ROUTINE

while 1:
	if len(paired)==12:
		print("Job Done!, shutting down... BYE BYE")
		exit()
	
	
	if silver == True: 
				
		dist,rot_y,code = find_silver_token()
	else:
		dist,rot_y, code, size = find_golden_token(actual_dist_offset)	

												
	if (dist==-1 or rot_y==-1) or (code in paired): #no valid token found situation
		
		if try_angle>=330: #if the robot has done a complete rotation while searching a valid token, we try to increase the reasearch distance
		
			actual_dist_offset= increase_research(actual_dist_offset)
			try_angle=0
 			 
		else:	
			try_angle, direction= look_around(try_angle, direction) #incrementing the searching angle in order to find a valid token
 			
 		

	elif dist<=d_th and silver==True: #grabbing situation
	
		print("Found Silver Token")
		try_angle=0	
		if R.grab():
			silver_grabbed=code #save the code of the grabbed token 
			print("Grabbed")
			silver=not silver
			actual_dist_offset=dist_offset	#reset the research distance for the next token, in case it has been increased to find the token already released 	
		else:
			print("Oh no! I can't grab it :(")	
		
		
		
	elif dist<=3.5*size and silver==False: #release situation
	
		try_angle=0
		print("Near enough to target Golden Token")	
		if R.release():
			gold_released=code #saving the code of the golden token near which the robot released the silver token  
			pairs=[silver_grabbed,gold_released]
			paired.extend(pairs) # save the token codes that are already paired
					
			print("Silver Token Released")
			print("Token already paired:",paired)
			
			drive(-30,1)
			turn_angle(130,1.5) #the robot turns back oriented to an anti-clockwise rotation
			silver=not silver
			actual_dist_offset=dist_offset #reset the research distance for the next token, in case it has been increased to find the token already released 
		else:
			print("Oh no! I can't Release it :(")						
		
		
					
	elif (rot_y>a_th or rot_y<-a_th) and (not code in paired): #next token disalignment situazion
		
		try_angle=0				
		turn_angle(rot_y,0.5)
				
				
	elif (-a_th<=rot_y<=a_th) and (not code in paired): #aligned to the next token situation
		
		try_angle=0	
		drive(60,dist/100.0)  #precision approaching movement 




    
    
