#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "assignment2_tarabotto/Custom.h"
#include "assignment2_tarabotto/Send.h"
#include "assignment2_tarabotto/Cancel.h"
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>


//GLOBAL VARIABLES

float p_x,p_y, vel_x,vel_z; //actual position and velocities
float goal_x,goal_y; //goal coordinates
bool cancel_flag; //flag that activates the abortion routine
bool goal_flag; //flag that activates the setting goal routine

//SERVICES

//takes the goal coordinates saving them into variables, giving also response of received coordinates and set appropriate flag to setting goal
bool send_coord(assignment2_tarabotto::Send::Request &req, assignment2_tarabotto::Send::Response &res)
{
	//DA COMPETARE
	goal_x = req.x;
	goal_y = req.y;	
	res.x = goal_x;
	res.y = goal_y;
	
	goal_flag=1;
	
	return true;
}

//sets cancel flag in order to start the abortion of the run and replies with a feedback of what happened
bool cancel_goal(assignment2_tarabotto::Cancel::Request &req, assignment2_tarabotto::Cancel::Response &res)
{
		
	cancel_flag=1;
	res.cancel_feedback=1;
	
	return true;
}



// /odom SUBSCRIBE saving actual position and velocities
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	
	p_x= msg->pose.pose.position.x;
	p_y= msg->pose.pose.position.y;
	vel_x= msg->twist.twist.linear.x;
	vel_z= msg->twist.twist.angular.z;
	
}



//MAIN

int main (int argc, char **argv)
{
	ros::init(argc, argv, "action_client"); //init node
	ros::NodeHandle nh; //init nodehandle
	
	
  // create the action client
  // true causes the client to spin its own thread
	actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("/reaching_goal", true);
  
  
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time
	
	//PUBLISHING
	ros::Publisher pub = nh.advertise<assignment2_tarabotto::Custom>("custom_topic",100);  
	
	//SUBSCRIBING
	ros::Subscriber sub2 = nh.subscribe("/odom", 100, odomCallback);  //nav_msgs::Odometry
	
	
	//SERVICES 
	ros::ServiceServer send_service=nh.advertiseService("/ac_send", send_coord); //instance of send service that calls send_coord function
	ros::ServiceServer cancel_service=nh.advertiseService("/ac_cancel", cancel_goal); //instance of cancel service that calls cancel_goal function

	//INSTANCES OF MESSAGES	
	assignment_2_2022::PlanningGoal goal; 
	assignment2_tarabotto::Custom odometry;

	
	ROS_INFO("Action server started.");
	
	
	//INFINITE WHILE LOOP
	while(ros::ok())
	{
		//Cancel Routine		
		if(cancel_flag==1)
		{
			ac.cancelGoal(); //cancel command
			
			cancel_flag=0; //reset flag
		}
		
		//Setting goal Routine                     
		if (goal_flag==1)
		{
			//filling request
			goal.target_pose.pose.position.x= goal_x; 
			goal.target_pose.pose.position.y= goal_y; 
			ROS_INFO("Sending goal.");
			ac.sendGoal(goal); //setting command
			
			goal_flag=0; //reset flag
		}
				
		
		//Filling the message Odometry
		ros::spinOnce();
		odometry.x= p_x;
		odometry.y= p_y;
		odometry.vel_x= vel_x;
		odometry.vel_z= vel_z;
		pub.publish(odometry); //publish current position and velocity needed by average_speed node

		


		ros::spinOnce();
	}
	
	return 0;
}  

	
