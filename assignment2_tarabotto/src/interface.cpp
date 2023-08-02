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

int state;

//CALLBACK

//update the current state variable with the status flag contained in the server message
void goalCallback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg)
{
		
	state= msg->status.status; 

}


//FUNCTIONS

//interface to insert the goal x coordinate
int insert_x()
{

	
	float x_insert=0.0;
	
	std::cout << "\n\nEnter desired X coordinate: ";
	std::cin >> x_insert;
	std::cin.clear();
	
 	
 	return x_insert;

}

//interface to insert the goal x coordinate
int insert_y()
{


	float y_insert=0.0;
	
	std::cout << "\nEnter desired Y coordinate : ";
	std::cin >> y_insert;
	std::cin.clear();
 	
 	
 	return y_insert;

}

//machine state interface that allow the user to cancel the goal
bool abortion_handler()
{
	
	std::string response; //temp string variable in which is saved the user response
	
	std::cout << "\nCoordinates have been sent, do you want to cancel? Type 'Y' or 'N' ";
	std::cin >> response;
	
	ros::spinOnce(); //update the status, in case the robot reached the goal while the user was writing the answer
	
	
	if ((response=="y" or response=="Y") and state!=3)
	{		
		std::cout<<"\n**Cancellation requested from the Client.**\n";
   		return 0;
	}
		
	else if ((response=="n" or response=="N") and state!=3)
	{
		std::cout<<"\n**Coordinates confirmed.**\n";
		return 1;
	}
		
		
	else if (state==3)
	{
		//don't consider any set action by the user because the robot has already finished the run
		std::cout<<"\nTHE ROBOT IS ALREADY ";
		return 1;
	}	
	
	else 
	{	
		//if the input is not what expected
		bool retry;
		std::cout<<"\n**Invalid input, retry**";
		retry= abortion_handler();
		return retry;
	}
		
}

//graphical waiting bar that digits and cancel dots in order to communicate the user that the robot is doing something
int show_progress (int i, std::string message)
{
	if(i==0)
	{
		std::cout<<"\n"<<message; //at the beginning show the message 
		i++;
	}
	
	else if(i>0 and i<=8) //start to put a dot every iteration
	{
		std::cout<<".";
		i++;
	}
	
	else if(i>8 and i<=16) //when there are 8 dots, it goes backward cancelling a dot every iteration 
	{
		std::cout<<"\b";
		std::cout<<" ";
		std::cout<<"\b";
  		i++;
	}
				
	else //if all the dots are cancelled, reset the position index in order to restart
	{
		i=1;
	}
	return i;
}



//MAIN

int main (int argc, char **argv)
{	
	std::cout << std::unitbuf; 
	ros::init(argc, argv, "interface"); //init node
	ros::NodeHandle nh; //init nodehandle
	
	//Declarations
	float x=0.0;
	float y=0.0;
	bool interface_flag; //state flag of user's decision, abortion_handler output
	bool cancel_feedback; //cancel flag set by the feedback of the service message used to cancel the the goal
  	
  	//SUBSCRIBING	
	ros::Subscriber sub = nh.subscribe("/reaching_goal/result", 100, goalCallback); //subscribe to result topic
	
	//SERVICES (client)
	ros::ServiceClient send_client =  nh.serviceClient<assignment2_tarabotto::Send>("/ac_send"); //used to call send service by passing the appropriate message
	ros::ServiceClient cancel_client =  nh.serviceClient<assignment2_tarabotto::Cancel>("/ac_cancel"); //used to call cancel service by passing the appropriate message
	
	//Service Messages
	assignment2_tarabotto::Send send_srv; //initialize the Send service message
	assignment2_tarabotto::Cancel cancel_srv; //initialize the Cancel service message 
	
	
	//INFINITE LOOP
	while(ros::ok())
	{
		state=10; //reset the state with a non-used flag value
		
		//insert coordinate call functions
		x = insert_x();
		y = insert_y();
		
		//compile request field of Send service message with the just received coordinates
		send_srv.request.x = x;
		send_srv.request.y = y;
		send_client.call(send_srv); //call the "send service" by sending the "Send" service message initialized before
		
		std::cout<< "\nGoal position set at: ("<<send_srv.response.x<< ";"<<send_srv.response.y<<")\n"; 
		
		
		interface_flag= abortion_handler();	// The interface asks the user to confirm or cancel the run, refreshing the appropriate flag
		ros::spinOnce(); //redundant
		
		
		//The User decided to not cancel the goal
		if (interface_flag==1 or (interface_flag==0 and state==3)) //2nd check redundant for security
		{	
			
			int i=0; //position index for waiting dots animation
			ros::Rate rate(2); //refreshing the state every 0.5 seconds in order to avoid the stream clogging for too much dots
			
			//"Not yet arrived" refreshing loop
			while (state!=3)
			{	
				
				ros::spinOnce();
				
				if(state==3)
				{
					std::cout<<"\n";
					break; //the refreshing stops in the exact moment the robot reaches the goal
				}
				
				i=show_progress(i, "The Robot is moving"); //calling the dots animation updating every time the index position
				rate.sleep();
			}
			
			std::cout << "ARRIVED AT DESTINATION";			
		}
		
		
		//The User decided to cancel the goal			
		else if(interface_flag==0 and state!=3) //2nd check redundant for security
		{
			
			cancel_client.call(cancel_srv); //call the cancel service by sending the Cancel service message initialized before
			cancel_feedback=cancel_srv.response.cancel_feedback; //save the cancel feedback from cancel message
			
			//resending cancel message until the robot cancels the goal
			while(cancel_feedback==0)
			{
				std::cout<<"\nError, retry to cancel";
				cancel_client.call(cancel_srv); //call the cancel service by sending the Cancel service message initialized before
				cancel_feedback=cancel_srv.response.cancel_feedback; //save the cancel feedback
			}
			
			int i=0; //position index for waiting dots animation
			ros::Rate rate(5);//setting refresh rate
			
			//"Not yet cancelled" refreshing loop
			while(state!=2)
			{	
				
				ros::spinOnce(); //update current state until it confirms that the goal is cancelled
				
				if(state==2)
				{
					std::cout<<"\n";
					break; //the refreshing stops in the exact moment the robot reaches the goal
				}
				
				i=show_progress(i,"Cancelling goal"); //calling the dots animation updating every time the index position
				rate.sleep();
			}	
			
			std::cout<<"RUN CANCELLED";
		}
		
			
	}
	
	return 0;
}	
