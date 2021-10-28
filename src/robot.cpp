/** @ package exprob_assignment1
* 
*  \file robot.cpp
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
*    
*/

#include "ros/ros.h"
#include "exprob_assignment1/MoveTo.h"
#include "std_msgs/Bool.h"
#include "exprob_assignment1/Oracle.h"
#include "exprob_assignment1/Hypothesis.h"
#include "std_msgs/String.h"
#include <string>

// global variables
int posx[9]={0,10,10,10,0,-10,-10, -10, -10 };
int posy[9]={10,10,0,-10,-10,-10,-5,5,10};
exprob_assignment1::Hypothesis message;
int behaviour=0;
//callback for the topic hypothesis
// change behavior and save hypothesis on global variable

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

void hypothesisCallback( const exprob_assignment1::Hypothesis x)
{ behaviour=1;
 message = x;	
}

int main( int argc, char **argv)
{
	ros::init(argc, argv, "robot");
	int index=0;
	int temp=0;
	int counter=0;

	char *ids={"ID2"};
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
		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << std::endl;
		client.call(p);
   		std::cout << "Position reached" << std::endl;
   		msg.data=true;
    reach_pub.publish(msg);
   		sleep(1);
	}

	if (behaviour==1)
	{ 
		p.request.x = 0;
		p.request.y = 0;
		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << std::endl;
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
			return 0;
		}

	}
	
}
}
