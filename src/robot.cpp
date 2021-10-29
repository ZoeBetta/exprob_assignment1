/** @ package exprob_assignment1
* 
*  \file robot.cpp
*  \brief This file implements the robot behaviour
*
*  \author Zoe Betta
*  \version 1.0
*  \date 26/10/2021
*  \details
*   
*  Subscribes to: <BR>
*	 /hypothesis
*
*  Publishes to: <BR>
*	 /reached
*
*  Services: <BR>
*    None
* 
*  Client Services: <BR>
*    /move_to
*    /oracle
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*    This node has two behaviours, in order to switch from the first behaviour
*    a new message should be received on the topic /hypothesis. If no hypothesis
*    is being detected the robot decides a random room to reach and calls
*    the service /move_to in order to reach the desired position. When the
*    position has been reached the node publishes on the topic /reached.
*    If a new hypothesis arrives on the topic the robot moves to the Home
*    position (0;0) and calls the service /oracle to find out if the received 
*    hypothesis is the winning one.
*/

#include "ros/ros.h"
#include "exprob_assignment1/MoveTo.h"
#include "std_msgs/Bool.h"
#include "exprob_assignment1/Oracle.h"
#include "exprob_assignment1/Hypothesis.h"
#include "std_msgs/String.h"
#include <string>
#include <ctime>

// global variables
int posx[9]={0,10,10,10,0,-10,-10, -10, -10 };
int posy[9]={10,10,0,-10,-10,-10,-5,5,10};
char *loc[9]={ "Hall", "Lounge" , "Dining Room", "Kitchen", "Ballroom" , "Conservatory", 
				"Biliard Room", "Library", "Study"};
exprob_assignment1::Hypothesis message;
int behaviour=0;

// function declaration
double randMToN(double M, double N);
void hypothesisCallback( const exprob_assignment1::Hypothesis x);

/**
 * \brief: main function
 * \param : None
 * 
 * \return: 0
 * 
 * This is the main function, it initializes the subscribers, publishers and
 * server. It also initializes the the seed for the random generation of 
 * numbers. If the behaviour is the first one the robot goes to a random 
 * position and publishes on a topic whe nthe position has been reached. 
 * If the behaviour is the second one it goes to home and then calls the /oracle 
 * server to check if the hypothesis is correct. If it is correct the node 
 * exits, if instead it is wrong the behaviour goes back to the previous one
 * to search for more hints.
 */
int main( int argc, char **argv)
{
	ros::init(argc, argv, "robot");
	int index=0;
	int temp=0;
	int counter=0;

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<exprob_assignment1::MoveTo>("/move_to_server");
	exprob_assignment1::MoveTo p;
	ros::NodeHandle n1;
    ros::Publisher reach_pub=n1.advertise<std_msgs::Bool>("/reached", 1000);
    std_msgs::Bool msg;
    ros::NodeHandle n2;
	ros::ServiceClient client_or = n2.serviceClient<exprob_assignment1::Oracle>("/oracle");
	exprob_assignment1::Oracle o;
	ros::NodeHandle n3;
	ros:: Subscriber reached= n3.subscribe("hypothesis", 1000, hypothesisCallback);
	srand(time(NULL));
	
	while(ros::ok()){
   	ros::spinOnce();
   	
   	
	if (behaviour==0)
	{ 
		temp=randMToN(0,8);
		while (temp==index)
		{
		temp=randMToN(0,8);
		}
		index=temp;
		p.request.x = posx[index];
		p.request.y = posy[index];
		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y <<"  " << loc[index] <<std::endl;
		client.call(p);
   		msg.data=true;
    reach_pub.publish(msg);
   		sleep(1);
	}

	if (behaviour==1)
	{ 
		p.request.x = 0;
		p.request.y = 0;
		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << "  Home"<< std::endl;
		client.call(p);
   		std::cout << "Position reached" << std::endl;
   		msg.data=true;

		o.request.id=message.ID;
		client_or.call(o);
		if ( o.response.ok==false)
		{
			std::cout << "wrong hypothesis" << std::endl;
			behaviour=0;
		}
		else {
			std::cout << "correct hypothesis" << std::endl;
			std::cout <<message.who<< " with the "<< message.what<<" in the "<<message.where<< std::endl;
			
			return 0;
		}

	}
	
}
}

/**
 * \brief: It generates a random number
 * \param M: the lower bound of the interval I want to have the random number in
 * \param N: the upper bound of the interval I want to have the random number in
 * 
 * \return: the random number generated
 * 
 * This function uses the library function rand() to generate a random number and
 *  then resizes it to be in the interval [M;N].
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * \brief: When new data are available on the topic /hypothesis.
 * \param x: of type Hypothesis.msg 
 * 
 * \return: None
 * 
 * When new hypothesis are available the global variable behaviour switches
 * to one and the data are saved on the global variable message.
 */
void hypothesisCallback( const exprob_assignment1::Hypothesis x)
{ behaviour=1;
 message = x;	
}
