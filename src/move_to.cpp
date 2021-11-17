/** @ package exprob_assignment1
* 
*  \file move_to.cpp
*  \brief It moves to the required position
*
*  \author Zoe Betta
*  \version 1.0
*  \date 26/10/2021
*  \details
*   
*  Subscribes to: <BR>
*	 None
*
*  Publishes to: <BR>
*	 None
*
*  Services: <BR>
*   /move_to
* 
*  Client Services: <BR>
*    None
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
* This node implements a server that receives a goal position and implements
* the algorithm to reach it. For simplicity it just implements a delay to
* simulate it.
*/

#include "ros/ros.h"
#include "exprob_assignment1/MoveTo.h"
#include "std_msgs/Bool.h"
#include <unistd.h>

// function declaration
bool move(exprob_assignment1::MoveTo::Request &req, exprob_assignment1::MoveTo::Response &res);

/**
 * \brief: main function
 * \param : None
 * 
 * \return 0
 * 
 * This is the main function, it initializes the subscribers, publishers and
 * server. 
 */
int main( int argc, char **argv)
{
	ros::init(argc, argv, "move_to");
	// definition of the node handles
    ros::NodeHandle n;
    // definition of the Server on the topic /move_to_server 
    ros::ServiceServer service= n.advertiseService("/move_to_server", move);
    // until ros is running correctly I wait for a request   	   
    while(ros::ok())
		{
			ros::spinOnce();
		}
	return 0;
}

/**
 * \brief: it implements the move algorithm
 * \param &req : the request for the service, the x and y coordinates 
 * 
 * \return &res : the answer that is sent back to the caller
 * 
 * This function implements the move_to algorithm, it just implements a
 *  delay in this case.
 */
bool move(exprob_assignment1::MoveTo::Request &req, exprob_assignment1::MoveTo::Response &res)
	{
		// when the server receives a request it sleeps for 1 second
		// this behaviour is implemented to simulate the time to reach the next room
		usleep(500000);
		// it sends the answer back to the client
		res.ok=true;
		return 1;
	}


