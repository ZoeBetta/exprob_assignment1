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
char *hint[13]={"ID1/who/green", "ID1/what/candlestick","ID1/where/conservatory", 
	"ID2/who/plum", "ID2/what/dagger","ID2/where/lounge",
	"ID3/who/mustard", "ID3/what/pipe","ID3/where/kitchen", "ID3/what/rope", 
	"ID4/who/peacock", "ID4/what/revolver","ID4/where/library",	
};
std_msgs::String winner;
std_msgs::String id_req;

//function declaration
double randMToN(double M, double N);
void reachCallback( const std_msgs::Bool x);
bool oracle(exprob_assignment1::Oracle::Request &req, exprob_assignment1::Oracle::Response &res);

/**
 * \brief: main function
 * \param : None
 * 
 * \return: 0
 * 
 * This is the main function, it initializes the subscribers, publishers and
 * server. It also initializes the the seed for the random generation of 
 * numbers.
 */
int main( int argc, char **argv)
{
	ros::init(argc, argv, "oracle");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::NodeHandle n2;
	ros:: Subscriber reached= n.subscribe("reached", 1000, reachCallback);
	hint_pub= n1.advertise<std_msgs::String>("/hint", 1000);
	ros::ServiceServer service= n2.advertiseService("/oracle", oracle);
	srand(time(NULL));
	ros:: spin();
	return 0;
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
 * \brief: called everytime a new message is published on the topic reached
 * \param x: the content of the published message.
 * 
 * \return: None
 * 
 * This function is called when new data is published. It then generates
 *  a random number between 1 and 10 and if that number is greater than 3 
 * a random hint is published on the topic hint.
 */
void reachCallback( const std_msgs::Bool x)
{
	
int index;
int rand;
int i= randMToN(1,10);
std_msgs::String msg;
if (i>3)
	{
		index=randMToN(0,13);
		msg.data=hint[index];
		hint_pub.publish(msg);
	}
}

/**
 * \brief: This is the server
 * \param  &req : it is the server request to be elaborated
 * \param & res : it is the responses to return to the client calling
 * 
 * \return: None
 * 
 * This function receives a request from a client with a string, it then checks
 * if the the request is the same as the winning ID, in that case it returns
 * true to the caller, false otherwise.
 */
bool oracle(exprob_assignment1::Oracle::Request &req, exprob_assignment1::Oracle::Response &res)
	{
		winner.data="ID4";
		id_req.data= req.id;
		if (id_req.data==winner.data)
		{
			res.ok= true;
		}
		else {
				res.ok=false;
		}
	}
