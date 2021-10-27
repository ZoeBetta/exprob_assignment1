/** @ package exprob_assignment1
* 
*  \file move_to.cpp
*  \brief 
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
*
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*/

#include "ros/ros.h"
#include "exprob_assignment1/MoveTo.h"
#include "std_msgs/Bool.h"


bool move(exprob_assignment1::MoveTo::Request &req, exprob_assignment1::MoveTo::Response &res)
	{
		std::cout << "MOVETO" << std::endl;
		res.ok=true;
		sleep(1);
		return 1;
	}

int main( int argc, char **argv)
{
	   ros::init(argc, argv, "move_to");
       ros::NodeHandle n;
    

    ros::ServiceServer service= n.advertiseService("/move_to_server", move);
       	   
       	   
    while(ros::ok()){
   	ros::spinOnce();

   	
   	}

   return 0;
	// when it is called it waits some time (1 sec)
	// it publishes on the topic reach new room
	// if position home do not send the topic reach
	// it returns reached to the caller
}
