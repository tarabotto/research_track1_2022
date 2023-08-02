#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "assignment2_tarabotto/Custom.h"
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

#define SAT 20 //dimension of the velocity array

//GLOBAL VARIABLES

float p_x,p_y, vel_x,vel_z, g_x,g_y;  //actual position, actual velocities, actual goal position
float velox[SAT]={0.0}; //array that will contains last instant liinear velocity samples
float veloz[SAT]={0.0}; //array that will contains last instant liinear velocity samples
int num1; //index that indicates number of samples inserted in the array
bool sat_flag=0; //flag that indicates when the array is full
float avg_x, avg_z; //average velocities



//CALLBACKS

//Gets the goal position coordinates from /reaching_goal/goal topic
void goalCallback(const assignment_2_2022::PlanningActionGoal::ConstPtr& msg)
{
	
	g_x=msg->goal.target_pose.pose.position.x;
	g_y=msg->goal.target_pose.pose.position.y;


}


//Gets the actual position and velocities of the robot from /custom_topic topic
void customCallback(const assignment2_tarabotto::Custom::ConstPtr& msg)
{

	p_x= msg->x;
	p_y= msg->y;
	vel_x= msg->vel_x;
	vel_z= msg->vel_z;
	
}



//FUNCTIONS

//Calculate the distance from the goal with Pythagorean theorem
float calculate_distance()
{
	float dist;
	dist=sqrt(pow((g_x-p_x),2)+pow((g_y-p_y),2));
	return dist;
}



//Calculate the average velocities by performing the average of the arrays data
void array_calculator()
{

		int den; //denominator of the average calculus
		float sum_x=0.0; //sum of linear velocity array data
		float sum_z=0.0; //sum of angular velocity array data
		int temp_index;
		
		//overwrite the arrays with new velocities
		velox[num1]=vel_x;
		veloz[num1]=vel_z;
		
		//update the array index
		num1++;
		
		if(sat_flag==0)
		{
			//if the array is not full, the average is calculated on the number of samples in the array (num1 samples)
			temp_index=num1;
			den=num1;
		}
		else 
		{
			//if the array is full, the average is calculated on the entire dimension of the array (SAT samples)
			temp_index=SAT;
			den=SAT;
		}
		
		//calculate the sum of array elements, if the array is not full yet the sum is performed with only the values added since the node started
		//otherwise,the sum is performed with all the array values
		for (int i=0; i<temp_index;i++)
		{	
		sum_x= sum_x + velox[i];
		sum_z= sum_z + veloz[i];
		}	

		//calculating averages
		avg_x= sum_x/den;
		avg_z= sum_z/den;

		if(num1>=SAT)
		{
			//when the array is fullfilled with the last sample, set the flag of "fullness" and reset the index to start overwriting samples		
			sat_flag=1;
			num1=0;
		}
		
		
}





int main (int argc, char **argv)
{
	
	ros::init(argc, argv, "average_speed"); //init node
	ros::NodeHandle nh; //init nodehandle

	//SUBSCRIBING 
	ros::Subscriber sub = nh.subscribe("/reaching_goal/goal", 10, goalCallback); 
	ros::Subscriber sub2 = nh.subscribe("/custom_topic", 10, customCallback);

	//Setting the publishing frequency by reading the parameter in the launch file
	float pub_freq;
	ros::param::get("publish_frequency", pub_freq);
	ros::Rate loop_rate(pub_freq);
	
	
	//Calculate and print infinite While loop
	
	while(ros::ok())
	{
		
		//Calling functions
		float distance= calculate_distance();
		array_calculator();
		

		
		std::cout << "\n\nThe distance from the goal is " << distance;
		std::cout << "\nAverage LINEAR velocity is: " << avg_x;
		std::cout << "\nAverage ANGULAR velocity is: " << avg_z<<"\n";
		
		ros::spinOnce();
		loop_rate.sleep();

	}

return 0;

}



