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
#include <unistd.h>

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
 * \return 0
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
	// variable definition
	int index=0;
	int temp=0;
	exprob_assignment1::MoveTo p;
	std_msgs::Bool msg;
	exprob_assignment1::Oracle o;
	// definition of the node handles
	ros::NodeHandle n;
	ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::NodeHandle n3;
    // definition of the Client for the Server on the topic move_to_server
	ros::ServiceClient client = n.serviceClient<exprob_assignment1::MoveTo>("/move_to_server");
	// definition of the publisher to the topic /reached
    ros::Publisher reach_pub=n1.advertise<std_msgs::Bool>("/reached", 1000);
    // definition of the Client for the Server on the topic oracle
	ros::ServiceClient client_or = n2.serviceClient<exprob_assignment1::Oracle>("/oracle");
	// definition of the subscriber to the topic hypothesis
	ros:: Subscriber hyp= n3.subscribe("hypothesis", 1000, hypothesisCallback);
	// defining the seed for the random generation of numbers
	// this is needed to avoid generating always the same numbers
	// it takes as input the time of the system as seed
	srand(time(NULL));
	// while ros is running correctly
	while(ros::ok())
		{
			ros::spinOnce();
			// check what the behaviour is set to, if it is zero I look for hints
			if (behaviour==0)
				{ 
					// find a random number to decide the random  position to go to
					temp=randMToN(0,8);
					// look for a new index until I find one that is different from the one of the position I am in now
					while (temp==index)
						{
							temp=randMToN(0,8);
						}
					// save the current index
					index=temp;
					// set the request for the server move_to_server for the room indicated by the index
					p.request.x = posx[index];
					p.request.y = posy[index];
					// print on the screen the requested position
					std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y <<"  " << loc[index] <<std::endl;
					// call the client for the server move_to_server
					client.call(p);
					// set the message for the reached publisher to true
					msg.data=true;
					// publish the message
					reach_pub.publish(msg);
					// wait 1 second to simulate the looking around for a hint
					usleep(500000);
				}
			// if behaviour is equal to 1 I go to the home position and ask the oracle for the solution
			if (behaviour==1)
				{ 
					// set the request for the move_to_server to (0;0) the home position
					p.request.x = 0;
					p.request.y = 0;
					// print on the screen the requested position
					std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << "  Home"<< std::endl;
					// call the client
					client.call(p);
					// set as a request for the oracle server the message ID received on the topic hypothesis
					o.request.id=message.ID;
					// call the oracle server
					client_or.call(o);
					// if the answer is false
					if ( o.response.ok==false)
						{
							// the hypothesis was the wrong one
							std::cout << "Wrong hypothesis" << std::endl;
							// go back to the behaviour 0, looking for hints
							behaviour=0;
						}
					// if the answer is true	
					else 
						{
							// print that the hypothesis was correct
							std::cout << "Correct hypothesis" << std::endl;
							// print the hypothesis
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
 * \return the random number generated
 * 
 * This function uses the library function rand() to generate a random number and
 *  then resizes it to be in the interval [M;N].
 */
double randMToN(double M, double N)
{     
	return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
}

/**
 * \brief: When new data are available on the topic /hypothesis.
 * \param x: of type Hypothesis.msg 
 * 
 * \return None
 * 
 * When new hypothesis are available the global variable behaviour switches
 * to one and the data are saved on the global variable message.
 */
void hypothesisCallback( const exprob_assignment1::Hypothesis x)
{ 
	// when it is received a new message the behaviour is changed
	behaviour=1;
	// save the message on a global variable
	message = x;	
}
