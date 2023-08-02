#include "ros/ros.h"
#include <assignment_2_2022/PlanningAction.h>
#include "assignment2_tarabotto/Count.h"
#include <stdlib.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>

//GLOBAL VARIABLES

int state; //actual status
int good_counter=0; //number of goal reached
int bad_counter=0; //number of goal failed

//CALLBACK

//update current state
void goalCallback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg)
{
	
	state= msg->status.status;
}


//SERVICE

//Print the amount of goal reached and failed and copies also into the response message
bool count_fails(assignment2_tarabotto::Count::Request &req, assignment2_tarabotto::Count::Response &res )
{	
		
	res.success=good_counter;
	res.cancel=bad_counter;
	ROS_INFO("\n Goals Reached: %i \n Goals Failed: %i \n", good_counter, bad_counter);
	
	
	return true;
	
}


//MAIN

int main (int argc, char **argv)
{ 
	
	ros::init(argc, argv, "counter"); //init node
	ros::NodeHandle nh; //init nodehandle
	
	//SERVICE
	ros::ServiceServer service=nh.advertiseService("/count", count_fails); //instance of send service that calls count_fails function
	
	//SUBSCRIBING
	ros::Subscriber sub = nh.subscribe("/reaching_goal/result", 100, goalCallback);

	//INFINITE LOOP	
	
	while (ros::ok())
	{
	//increase appropriate counter basing on current status	
		if(state==3)
			{
				good_counter++;
				state=0; //reset current status so it will modified next time it changes on topic
			}
	
			else if(state==2)
			{
				bad_counter++;
				state=0; //reset current status so it will modified next time it changes on topic
			}	

		ros::spinOnce();

	}
	
	return 0;

}	
