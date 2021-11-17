# EXPERIMENTAL ROBOTICS LABORATORY - ASSIGNMENT 1
## BRIEF INTRODUCTION 
This project implements the game of Cluedo played by a robot that has to explore an unknown environment searching for hints. The robot should move in every room to look for hints and when a new hypothesis is ready to be made it should return to the starting location and ask if the hypothesis is correct.
## SOFTWARE ARCHITECTURE
![architecture of the software](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/Architecture.jpg?raw=true)  

This architecture is composed of four nodes. Three of them are written in c++ language (robot, oracle and moveto) and the remaining one in python (hint). The communications between the different nodes are  implemented either as ros publisher subscriber, when we do not want to guarantee the synchronization between the nodes, or as ros services and clients when we need the actions to be synchronized. The only node that communicates with the ontology is the hint node that, using the armor server, is able to access and modify the ontology.  

![state diagram for the program](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/RobotStateDiagram.jpg?raw=true)  

At the beginning of the program the robot is in the state move to. If no hypothesis is received the robot moves to a random room looking for hints. From there if a new hint is received the hint is elaborated and the ontology checked for new hypothesis. AFter that, even if no hint is received the robot returns to the move state to either move to a new random room or move home if an hypothesis is retrieved. If the robot received an hypothesis it moves home and when reached it goes to the state asking if the hypothesis is correct; from there if the hypothesis is correct it stops because the goal has been reached, if instead the hypothesis is wrong it goes back to the move to state.  

![sequence diagram for the program](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/SequenceDiagram.jpg?raw=true)  

At the beginnign the robot needs to move to a new random room. To do so it sends a request to the server MoveTo implemented in the moveto node. When the robot node receives the response that it reached the room it publishes true on the topic /reached, said topic is read by the subscriber in the node oracle. The oracle then randomly decides wheater to send a hint or not ( it generates a random number between 1 and 10 and if the number is grater than 3 it sends a hint ) and then randomly decides which hint to send. The hint is then sent with the publisher on the topic /hint, the node hint implements the subscriber for the topic /hint. To be noticed is that there is not the need for the robot to synchronize to the generation of hints, the robot waits a given amount of time ( in this case 0.5 second ) to simulate the activity of searching for hints in a room and then moves to another room. When a hint is received on the topic /hint the hint.py node elaborates it and checks in the ontology if a new hypothesis can be made, in case no new hypothesis can be made it just starts waiting again for a new hint. If instead an hypothesis can be made it publishes it on the topic /hypotesis that is read by the subscriber implemented in the robot node. The robot node then makes a request to the service implemented in the oracle node to discover if the hypothesis received is the correct one. If the node answers true than the program ends, if instead the hypothesis is wrong the robot keeps searching for new hints.

## INSTALLATION AND RUNNING PROCEDURE
In order to run this code it is necessary to have istalled in the same workspace the package armor that allows for the comunication and the handling of the ontology. Once the package is in the workspace it is mandatory to build it using the instruction in the github repository at the following link:
In particular it is necessary to run the following command `./gradlew deployApp`  in the armor folder.
Also before running the program is good practice to check if the ontology file is present in the package folder and if the path specified in the file hint.py ( that can be found inside the folder scripts ) is correct. The place where to change the path is signalled by two lines of code in all caps that specify that the following variable is the path.
If there is the need to change the hints that can be received they are defined in the file oracle.cpp in the folder src. The hints are formatted in the following way: each hint is a single string ID/class_type/name. Where class type can be who, what or where. If the total number of hint is changed be sure to change the dimension of the array and the maximum random number to be generated. Both these things are signalled by a line of code in all caps in correspondance to the line to change. 
Once completed all the previous steps to run the code it is sufficent to run in the workspace directory:
1. catkin_make -> to build the code and the dependencies
2. roslaunch exprob_assignment1 sim.launch -> to run the actual code 
When the process robot finishes it is necessary to end the program manually by pressing Ctr+C.
## HOW THE CODE RUNS
![diagram of the locations of the rooms](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/Locations.jpg?raw=true)  
In the image above a schematic representation of the locations that are reachable by the robot in the program. The name of the places and even the coordinates are inspired by the Cluedo Game Board. For what concerns the persons and the weapons I used the names present in the game. The complete list of all the possible elements of hints is the following:
- Suspected Person
  * Rev. Green
  * Prof. Plum
  * Col. Mustard
  * Mrs. Peacock
  * Miss. Scarlet
  * Mrs. White
- Probable Implements
  * Candlestick
  * Dagger
  * Lead Pipe
  * Revolver
  * Rope
  * Spanner
- Suspected Scene of Murder
  * Conservatory
  * Lounge
  * Kitchen
  * Library
  * Hall
  * Study
  * Ballroom
  * Dining Room
  * Billiard Room  
  
All the possible hints that can be send are declared in the oracle node and there are hints whith the correct number of elements ( one person, one place, one weapon ), hint with more elements than required, hints with less elements than required. Hints are declared as a string that cointains the ID of the hint, the type of hint and the name of the specific instance, the elements are divided by the character '/'; for example "ID5/what/Rope".  
In the following image we can see the normal output produed by the system while it is running:  
![Normal output behaviour](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/NormalBehaviour.jpg?raw=true)  
We can notice that the robot declares the requested position and synchronously it might receive an hint, the ID of the received hint is printed on the screen by the node hint. This synchronization is not forced and it is not needed for the correct behaviour of the architecture, it happens sinche the waiting times are long. By decreasing the waiting times, to for example 20 milliseconds we can notice that the hints received are no more synchronized and, when the system stops moving because the correc hypothesis is produced the robot stops moving but some hints keep being processed since there were more hints than the one processed and so the system fills the queue and before stopping it empties it. When a hypothesis is found that is both complete and consistent the hypothesis is printed on the screen and sent to the robot node. The robot moves to the Home location, at coordinates (0,0) and makes a request to the oracle server and prints if the hypothesis is correct or not. If it is wrong the robot starts moving again in other rooms to look for other hints. We can also notice that we don't receive a hint everytime the robot reaches a room, in those cases there is not anID in correspondance of a location to be reached.  
In the following picture what happens when the correct hypothesis is sent.  
![Output for correct hypothesis](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/Final.jpg?raw=true)  
When an hypothesis is correct the system replies with correct hypothesis and prints on the screen the hypothesis presented in the classical Cluedo way. At that point the robot nodes finishes and stops.  
I noticed that even if I had hints that were created with more than the correct amount of entities, if the system receives first three elements that are coherent ( one person, one weapon, one place ) it is able to send the hypothesis and check its correctness. For example the hint identified by ID8 is composed of: "ID8/who/Col. Mustard", "ID8/what/Candlestick","ID8/where/Dining Room", "ID8/who/Miss. Scarlet"; and so two possible suspects instead of one, it still sent the hypothesis and retrieves from it.  
![Output for wrong hypothesis](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Images/ID8.jpg?raw=true)  
## WORKING HYPOTHESIS

### SYSTEM'S FEATURES

### SYSTEM LIMITATIONS

### POSSIBLE TECHNICAL IMPROVMENTS

## CONTACTS
Author: Zoe Betta  

E-mail: zoe.betta@gmail.com  

E-mail: s5063114@studenti.unige.it
