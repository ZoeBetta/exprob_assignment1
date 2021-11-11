/** @ package exprob_assignment1
* 
*  \file oracle.cpp
*  \brief It sends random hints and checks if the hypothesis is the correct one
*
*  \author Zoe Betta
*  \version 1.0
*  \date 26/10/2021
*  \details
*   
*  Subscribes to: <BR>
*	 reached
*
*  Publishes to: <BR>
*	 hint
*
*  Services: <BR>
*    /oracle
* 
*   Client Services: <BR>
*    None
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
* This node subscribes to the reached topic; once a new message is published
* the node randomply decided wheather to publish the hint or not. The node
* also implements a server that when called checks if the ID received is the
* same as the winner's ID, in that case it sends back true, if the ID is 
* different it send back false. 
* In this node is defined the list of possible hints and the winning ID.
*  Be careful while adding or removing hints to change also the maximum
* random number to be generate to span the entire dimension of the hints array.
* It is signalled in the code where to change it. 
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <cstdlib>
#include "exprob_assignment1/Oracle.h"
#include "string.h"
#include <ctime>

//global variables
ros::Publisher hint_pub;
// BE CAREFUL IF YOU CHANGE THE NUMBER OF ELEMENTS CHANGE THE DIMENSION HERE
// AND ALSO THE MAXIMUM NUMBER THAT CA BE GENERATED BELOW
char *hint[31]={"ID1/who/green", "ID1/what/candlestick","ID1/where/conservatory", 
	"ID2/who/Plum", "ID2/what/dagger","ID2/where/lounge",
	"ID3/who/Mustard", "ID3/what/pipe","ID3/where/kitchen", "ID3/what/rope", 
	"ID4/who/Peacock", "ID4/what/revolver","ID4/where/library",	
	"ID5/who/Scarlet", "ID5/what/rope",	
	"ID6/who/White", "ID6/what/spanner","ID6/where/hall", "ID6/where/study",	
	"ID7/who/Plum", "ID7/what/dagger","ID7/where/ballroom",	
	"ID8/who/Mustard", "ID8/what/candlestick","ID8/where/diningroom", "ID8/who/Scarlet",
	"ID9/who/White", "ID9/what/pipe",	
	"ID10/who/Green", "ID10/what/revolver","ID10/where/conservatory",		
};

//function declaration
double randMToN(double M, double N);
void reachCallback( const std_msgs::Bool x);
bool oracle(exprob_assignment1::Oracle::Request &req, exprob_assignment1::Oracle::Response &res);

/**
 * \brief: main function
 * \param : None
 * 
 * \return 0
 * 
 * This is the main function, it initializes the subscribers, publishers and
 * server. It also initializes the the seed for the random generation of 
 * numbers.
 */
int main( int argc, char **argv)
{
	ros::init(argc, argv, "oracle");
	// definition of the node handles
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::NodeHandle n2;
	// definition of the subscriber to the topic reached
	ros:: Subscriber reached= n.subscribe("reached", 1000, reachCallback);
	// definition of the publisher to the topic /hint
	hint_pub= n1.advertise<std_msgs::String>("/hint", 1000);
	// definition of the Server on the topic /oracle 
	ros::ServiceServer service= n2.advertiseService("/oracle", oracle);
	// defining the seed for the random generation of numbers
	// this is needed to avoid generating always the same numbers
	// it takes as input the time of the system as seed
	srand(time(NULL));
	ros:: spin();
	return 0;
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
 * \brief: called everytime a new message is published on the topic reached
 * \param x: the content of the published message.
 * 
 * \return None
 * 
 * This function is called when new data is published. It then generates
 *  a random number between 1 and 10 and if that number is greater than 3 
 * a random hint is published on the topic hint.
 */
void reachCallback( const std_msgs::Bool x)
{
	// definition of variables
	int index;
	//int rand;
	// random number from 1 to 10 to decide randomly whether to send a hint or not
	int i= randMToN(1,10);
	// definition of the message that might be sent
	std_msgs::String msg;
	// if the random generated number is greater than 3 I send the message
	if (i>3)
		{
			// index of the hint to send. It is randomly generated from 0 to 13
			index=randMToN(0,31);
			// set the message data as the hint
			msg.data=hint[index];
			// publish the hint
			hint_pub.publish(msg);
		}
}

/**
 * \brief: This is the server
 * \param  &req : it is the server request to be elaborated
 * \param & res : it is the responses to return to the client calling
 * 
 * \return None
 * 
 * This function receives a request from a client with a string, it then checks
 * if the the request is the same as the winning ID, in that case it returns
 * true to the caller, false otherwise.
 */
bool oracle(exprob_assignment1::Oracle::Request &req, exprob_assignment1::Oracle::Response &res)
	{
		// define the winner ID
		std_msgs::String winner;
		// here manually set the ID of the winning hint
		// IF YOU WANT TO CHANGE IT BE SURE TO CHECK IT IS A COMPLETE AND CONSISTENT HINT
		// THEN YOU CAN CHANGE THE winner.data VARIABLE
		winner.data="ID4";
		if (req.id==winner.data)
		{
			res.ok= true;
		}
		else {
				res.ok=false;
		}
	}
