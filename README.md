# EXPERIMENTAL ROBOTICS LABORATORY - ASSIGNMENT 1
## BRIEF INTRODUCTION 
This project implements the game of Cluedo played by a robot that has to explore an unknown environment searching for hints. The robot should move in every room to look for hints and when a new hypothesis is ready to be made it should return to the starting location and ask if the hypothesis is correct.
## SOFTWARE ARCHITECTURE
![architecture of the software](https://github.com/ZoeBetta/exprob_assignment1/blob/main/Architecture.jpg?raw=true)

![state diagram for the program](https://github.com/ZoeBetta/exprob_assignment1/blob/main/RobotStateDiagram.jpg?raw=true)
![sequence diagram for the program](https://github.com/ZoeBetta/exprob_assignment1/blob/main/SequenceDiagram.jpg?raw=true)

## INSTALLATION AND RUNNING PROCEDURE
In order to run this code it is necessary to have istalled in the same workspace the package armor that allows for the comunication and the handling of the ontology. Once the package is in the workspace it is mandatory to build it using the instruction in the github repository at the following link:
In particular it is necessary to run the following command ____ in the ___ folder.
Also before running the program is good practice to check if the ontology file is present in the package folder and if the path specified in the file hint.py ( that can be found inside the folder scripts ) is correct. The place where to change the path is signalled by two lines of code in all caps that specify that the following variable is the path.
If there is the need to change the hints that can be received they are defined in the file oracle.cpp in the folder src. The hints are formatted in the following way: each hint is a single string ID/class_type/name. Where class type can be who, what or where. If the total number of hint is changed be sure to change the dimension of the array and the maximum random number to be generated. Both these things are signalled by a line of code in all caps in correspondance to the line to change. 
Once completed all the previous steps to run the code it is sufficent to run in the workspace directory:
1. catkin_make -> to build the code and the dependencies
2. roslaunch exprob_assignment1 sim.launch -> to run the actual code 

## HOW THE CODE RUNS

## WORKING HYPOTHESIS

### SYSTEM'S FEATURES

### SYSTEM LIMITATIONS

### POSSIBLE TECHNICAL IMPROVMENTS

## CONTACTS
Author: Zoe Betta  

E-mail: zoe.betta@gmail.com  

E-mail: s5063114@studenti.unige.it
