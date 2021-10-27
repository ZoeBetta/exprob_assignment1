/** @ package exprob_assignment1
* 
*  \file oracle.cpp
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
//#include "exprob_assignment1/MoveTo.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <cstdlib>
#include "exprob_assignment1/Oracle.h"
#include "string.h"

void reachCallback( const std_msgs::Bool x);
ros::Publisher hint_pub;

char *hint[12]={"ID1/who/green", "ID1/what/candlestick","ID1/where/conservatory", 
	"ID2/who/plum", "ID2/what/dagger","ID2/where/lounge",
	"ID3/who/mustard", "ID3/what/pipe","ID3/where/kitchen",
	"ID4/who/peacock", "ID4/what/revolver","ID4/where/library",
	
};

std_msgs::String winner;
std_msgs::String id_req;
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

void reachCallback( const std_msgs::Bool x)
{
	
int index;
int rand;
int i= randMToN(1,6);
std_msgs::String msg;
if (i>3)
	{
		index=randMToN(0,11);
		msg.data=hint[index];
		hint_pub.publish(msg);
		std::cout << "oracolo presente" << std::endl;
	}
}

bool oracle(exprob_assignment1::Oracle::Request &req, exprob_assignment1::Oracle::Response &res)
	{
		winner.data="ID2";
		id_req.data= req.id;
		if (id_req.data==winner.data)
		{
			res.ok= true;
		}
		else {
				res.ok=false;
		}
	}


int main( int argc, char **argv)
{
	ros::init(argc, argv, "oracle");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::NodeHandle n2;
	ros:: Subscriber reached= n.subscribe("reached", 1000, reachCallback);
	hint_pub= n1.advertise<std_msgs::String>("/hint", 1000);
	ros::ServiceServer service= n2.advertiseService("/oracle", oracle);
	// when the server is called
	// if ID==ID*
	// return 1
	// else 
	// return 0 
	ros:: spin();
	return 0;
}
