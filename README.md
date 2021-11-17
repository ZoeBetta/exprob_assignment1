# EXPERIMENTAL ROBOTICS LABORATORY - ASSIGNMENT 1
## BRIEF INTRODUCTION 
This project implements the game of Cluedo played by a robot that has to explore an unknown environment searching for hints. The robot should move in every room to look for hints and when a new hypothesis is ready to be made it should return to the starting location and ask if the hypothesis is correct.
## SOFTWARE ARCHITECTURE
![architecture of the software](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Architecture.jpg?raw=true)  

This architecture is composed of four nodes. Three of them are written in c++ language (robot, oracle and moveto) and the remaining one in python (hint). The communications between the different nodes are  implemented either as ros publisher subscriber, when we do not want to guarantee the synchronization between the nodes, or as ros services and clients when we need the actions to be synchronized. The only node that communicates with the ontology is the hint node that, using the armor server, is able to access and modify the ontology.  

![state diagram for the program](https://github.com/ZoeBetta/exprob_assignment1/blob/main/RobotStateDiagram.jpg?raw=true)  

At the beginning of the program the robot is in the state move to. If no hypothesis is received the robot moves to a random room looking for hints. From there if a new hint is received the hint is elaborated and the ontology checked for new hypothesis. AFter that, even if no hint is received the robot returns to the move state to either move to a new random room or move home if an hypothesis is retrieved. If the robot received an hypothesis it moves home and when reached it goes to the state asking if the hypothesis is correct; from there if the hypothesis is correct it stops because the goal has been reached, if instead the hypothesis is wrong it goes back to the move to state.  

![sequence diagram for the program](https://github.com/ZoeBetta/exprob_assignment1/blob/main/SequenceDiagram.jpg?raw=true)  

At the beginnign the robot needs to move to a new random room. To do so it sends a request to the server MoveTo implemented in the moveto node. When the robot node receives the response that it reached the room it publishes true on the topic /reached, said topic is read by the subscriber in the node oracle. The oracle then randomly decides wheater to send a hint or not ( it generates a random number between 1 and 10 and if the number is grater than 3 it sends a hint ) and then randomly decides which hint to send. The hint is then sent with the publisher on the topic /hint, the node hint implements the subscriber for the topic /hint. To be noticed is that there is not the need for the robot to synchronize to the generation of hints, the robot waits a given amount of time ( in this case 1 second ) to simulate the activity of searching for hints in a room and then moves to another room. When a hint is received on the topic /hint the hint.py node elaborates it and checks in the ontology if a new hypothesis can be made, in case no new hypothesis can be made it just starts waiting again for a new hint. If instead an hypothesis can be made it publishes it on the topic /hypotesis that is read by the subscriber implemented in the robot node. The robot node then makes a request to the service implemented in the oracle node to discover if the hypothesis received is the correct one. If the node answers true than the program ends, if instead the hypothesis is wrong the robot keeps searching for new hints.

## INSTALLATION AND RUNNING PROCEDURE
In order to run this code it is necessary to have istalled in the same workspace the package armor that allows for the comunication and the handling of the ontology. Once the package is in the workspace it is mandatory to build it using the instruction in the github repository at the following link:
In particular it is necessary to run the following command `./gradlew deployApp`  in the armor folder.
Also before running the program is good practice to check if the ontology file is present in the package folder and if the path specified in the file hint.py ( that can be found inside the folder scripts ) is correct. The place where to change the path is signalled by two lines of code in all caps that specify that the following variable is the path.
If there is the need to change the hints that can be received they are defined in the file oracle.cpp in the folder src. The hints are formatted in the following way: each hint is a single string ID/class_type/name. Where class type can be who, what or where. If the total number of hint is changed be sure to change the dimension of the array and the maximum random number to be generated. Both these things are signalled by a line of code in all caps in correspondance to the line to change. 
Once completed all the previous steps to run the code it is sufficent to run in the workspace directory:
1. catkin_make -> to build the code and the dependencies
2. roslaunch exprob_assignment1 sim.launch -> to run the actual code 

## HOW THE CODE RUNS
![diagram of the locations of the rooms](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Locations.jpg?raw=true)
## WORKING HYPOTHESIS

### SYSTEM'S FEATURES

### SYSTEM LIMITATIONS

### POSSIBLE TECHNICAL IMPROVMENTS

## CONTACTS
Author: Zoe Betta  

E-mail: zoe.betta@gmail.com  

E-mail: s5063114@studenti.unige.it
