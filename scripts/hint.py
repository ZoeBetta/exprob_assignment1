#! usr/bin/env python2

## @package exprob_assignment1
#
#  \file hint.py
#  \brief This program elaborates the hints received
#
#  \author Zoe Betta
#  \version 1.0
#  \date 29/10/2021
#  \details
#  
#  Subscribes to: <BR>
#       /hint
#
#  Publishes to: <BR>
#	    /hypothesis
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#       armor_interface_srv
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    This node receives the hints published on the topic /hint. Then it is 
#    able to elaborate them: it check if the person, location or weapon is 
#    already present in the ontology. In case it is not in the ontology yet
#    it adds it. After that it checks if the hint has already been saved in
#    the ontology, if it is the first time the hint has been received it 
#    checks if the hypothesy that correspond to that hint ID is complete 
#    and not inconsistent. In that case it checks if the hypothesy has 
#    already been sent, if not it publish the hipothesy on the topic /hypothesis
#    that is of type Hypothesis.msg. 

import copy
import math
import sys
import time
import geometry_msgs.msg
from exprob_assignment1.msg import Hypothesis
import numpy as np
import rospy
from std_msgs.msg import String
from armor_msgs.msg import * 
from armor_msgs.srv import * 

#global variables
people=[]
weapons=[]
locations=[]
hypothesis=[]
armor_service = None
pub= None

##
#	\brief This function implements the ros node
#	\param : None
#	\return : None
# 	
#   When the node is first initialized it loads all of the publishers, 
#   subscribers and servers. It also loads the ontology file needed to 
#   reason on the hints.
#	
def main():
  global  armor_service, pub
  rospy.init_node('Init')
  # definition of the Client for the Server on the topic armor_interface_srv
  armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
  # definition of the publisher to the topic /hypothesis
  pub=rospy.Publisher('/hypothesis', Hypothesis, queue_size=10)
  # waits for the service to be correctly running
  rospy.wait_for_service('armor_interface_srv')
  # definition of the subscriber to the topic hint
  sub_odom = rospy.Subscriber('/hint', String, clbk_hint)
  # load the ontology from the ontology file
  load_ontology()
  rospy.spin() 

##
#	\brief This function is called when new data are available on the topic /hint
#	\param msg: he data received on the topic /hint, it is of type std_msgs::String
#	\return : None
# 	
#	This function elaborates all of the hints received and decides weather 
#   or not to make an hypothesy. It checks if the hint is already been saved
#   into the ontology and after that if it is complete( at least one value
#   for each field who what where ) and not inconsistent ( not more than one
#   value for each field) if the hypothesy is complete and not inconsistent, 
#   and if it was never published before it is sent on the topic /hypothesis
#   as a message of type Hypothesis.msg, that has 4 fields each of string type:
#   ID, who, what, where.
def clbk_hint(msg):
    global hypothesis
    already_done=0
    # casting of the received message to the type string
    s=str(msg.data)
    # split the received data in the correspondance of the character '/'
    # it creates a list of strings
    hint_received=s.split('/')
    # set the ROS parameter to the ID of the hint just received
    rospy.set_param('ID', hint_received[0])
    print(rospy.get_param('ID'))
    # check if the hint was already received and so it is already saved in the ontology
    find=check_if_received_before(hint_received)
    # if it was never received before
    if find==0:
        # add to the ontology
        add_instance(hint_received[2],hint_received[1])
    # check if the hint is already in the hypothesis in the ontology	    
    check_in_ontology(hint_received[0], hint_received[1], hint_received[2])
    # check if the hypothesis is complete and consistent
    send=check_complete_consistent(hint_received[0])
    # if the hypothesis is both complete and consistent
    if send==1:
        # check if the hypothesis has already been sent, the sent hypothesis are in a global list
        for i in range(len(hypothesis)):
            if hint_received[0]==hypothesis[i]:
                already_done=1
        # if the hypothesis has not already been sent, send it to the robot
        if already_done==0:
            print('send to robot')
            # add the ID to the list of hypothesis sent
            hypothesis.append(hint_received[0])
            message= Hypothesis()
            # initialize the hypothesis to send
            message.ID=hint_received[0]
            temp=look_hypothesis(hint_received[0], 'who')
            message.who=temp[0]
            temp=look_hypothesis(hint_received[0], 'what')
            message.what=temp[0]
            temp=look_hypothesis(hint_received[0], 'where')
            message.where=temp[0]
            print(message)
            # send on the topic hypothesis
            pub.publish(message)
    else:
        # if the hypothesis is not complete or not consistent print to screen
        print (' not complete or not consistent')

##
#	\brief This function loads the ontology
#	\param : None 
#	\return : None
# 	
#	It loads the ontology from a file called cluedo_ontology in a specific
#   path. Be careful if you want to change the path of the file it needs
#   to be done here. The file is loaded by the proper call to the armor server.
def load_ontology():
    try:
        # set the request for the armor server to load ontology
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''
        # IT IS NECESSARY TO CHANGE THE FOLLOWING LINE IF THE FILE IS NOT THERE
        # CHANGE THE FIRST ARGUMENT WITH THE CORRECT PATH
        req.args= ['/root/ros_ws/src/exprob_assignment1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        # send the message on the server
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)
 
##
#	\brief This function adds an instance in the ontology
#	\param name : it is a string representing the name of the instance
#   \param class_type: it is a string representing the type of the class,
#                       it can be who, what or where
#	\return : None
# 	
#	 This function adds one entity to the ontology. First it checks which
#    class the entity belongs to and then it adds it to the ontology by
#    making the proper request to the armor server. After that it reasons and
#    specify that the new entity is not equal to any previous entities of
#    the same class      
def add_instance(name, class_type):
    try:
        # from the class type (who, what,where) find the class (PERSON,LOCATION,WEAPON)
        class_id=find_type(class_type)
        # set the request for the armor server to add an instance of a class
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, class_id]
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
        # Use the reasoner of the ontlogy
        reason()
        # Specify the element of the class are different, disjoint
        disjoint(class_id)
        # Use the reasoner of the ontlogy
        reason()
    except rospy.ServiceException as e:
        print(e)

##
#	\brief this function returns the class name corresponding to the given type
#	\param class_type : it is a string, it can be who, what or where 
#	\return : The class name
# 	
#	This function checks the type of the class given as input parameter and
#   returns the string with the corresponding class name.
def find_type(class_type):
    if class_type=='who':
        return 'PERSON'
    if class_type== 'what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION' 

##
#	\brief this function updates the ontology
#	\param : None
#	\return : None
# 	
#	 This function calls the armor server in order to run the reasoner
#    and make implicit knowledge explicit       
def reason():
    try:
        # set the request for the armor server to use the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)		
 
##
#	\brief This function specifies that all elements of a class are different
#	\param class : of type string it is the class of which element I want to make disjoint
#	\return : None
# 	
#	This function calls the armor server and by sending specific commands it
#   specifies that all entities inside the class passed as parameter are 
#   disjoint and different   
def disjoint(class_id):
    try:
        # set the request for the armor server to use the reasoner
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [class_id]
        # send the request
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e)        

##
#	\brief This function cleans the query returned from the ontology
#	\param query: the list of strings that needs to be cleaned
#	\return : query, the cleaned query
# 	
#	This function, for each element of the list passed as input it splits
#   the strings at the char '#' and takes only what is after. Then it removes
#   the last element of the remaing string. The received query is of type 
#   <http://www.emarolab.it/cluedo-ontology#rope> and in this example
#   we want to extract rope
def clean_queries(query):
    # for every element of the list received as input 
    for i in range(len(query)):
        temp=query[i]
        # split at the character '#'
        temp=temp.split('#')
        # save the lenght of the list returned after the split
        index=len(temp)
        # take only the last element ( lenght -1 ) 
        temp=temp[index-1]
        # saves it in the query, overwriting the one received and 
        # eliminating the last character
        query[i]=temp[:-1]
    return query

##
#	\brief This function checks if the entity has already been received
#	\param data: it is a list of strings I want to check if it has been received before
#	\return : find an integer that is 1 if the entity was already received
#             at least once and 0 if the entity was not received before.
# 	
#	 In this function, depending on the type of string that has been received, if 
#    a person, a location or a weapon, it checks in the corresponding global list 
#    if already present. If that is not the case it then adds it at the end.   
def check_if_received_before(data):
    global people, weapons, locations
    find=0
    i=0
    j=0
    k=0
    # if the data received is of class_type 'who'
    if data[1]=='who':
        # checks every element of the array people to see if the data received is already saved there
        for i in range(len(people)):
            if people[i]==data[2]:
                find=1;
        # if it is not there
        if find==0:
            # I add it to the global list
            people.append(data[2])
    # if the data received is of class_type 'what'
    if data[1]=='what':
        # checks every element of the array weapons to see if the data received is already saved there
        for j in range(len(weapons)):
            if weapons[j]==data[2]:
                find=1;
        # if it is not there
        if find==0:
            # I add it to the global list
            weapons.append(data[2])
    # if the data received is of class_type 'where'
    if data[1]=='where':
        # checks every element of the array locations to see if the data received is already saved there
        for k in range(len(locations)):
            if locations[k]==data[2]:
                find=1;
        # if it is not there
        if find==0:
            # I add it to the global list
            locations.append(data[2])
    return find            

##
#	\brief This function adds an hypothesis to the ontology
#	\param ID : of type string it is the ID of the hypothesis I want to add to
#	\param class_type : string representing the type of information I want to add
#   \param name : of type string it is the name of the information I want to add
#	\return : None
# 	
#	By calling the armor server with the proper commands I add an entity to
#   a given hypothesis. 
def add_hypothesis(ID,class_type,name):
    try:
        # set the request for the armor server to add an entity to an hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID,name]
        # send the request
        msg = armor_service(req)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)	

##
#	\brief It retrieves from an hypothesis a field
#	\param ID : of string type it is the hypothesis I want to check 
#   \param class_type : the property of the hypothesis that I want to retrieve
#	\return : res_final a string with the name of the entity I retrieved
# 	
#	This funciton calls the armor server to see in a given hypothesis identified
#   by its ID one field, identified by the class_type.
def look_hypothesis(ID,class_type):
    try:
        # set the request for the armor server to check one field (identified by class_type)
        # of an hypothesis (identified by an ID)
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID]
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        return res_final
    except rospy.ServiceException as e:
        print(e)   

##
#	\brief It checks if the hint received is already saved in the hypothesis
#	\param ID: of type string, the ID of the hypothesis I want to check 
#	\param class_type: of type string, the type of instance I want to check
#       if already present
#	\param name : of type string, the name of the entity I want to check
#	\return : None
# 	
#	This function checks if a hint ( composed of an ID, class_type and name) 
#   is already present in an hypothesis. If it is not present it adds it 
#   to the ontology.
def check_in_ontology(ID,class_type,name):
    try:
        namef=[]
        # it retrieves from the hypothesis ID the field identified by class_type
        res_final=look_hypothesis(ID, class_type)
        # add the name to namef
        namef.append(name)
        # if the name retrieved is different from the name received I add the hypothesis
        # it adds it even if the retrieved field is empty
        if res_final != namef:
            add_hypothesis(ID,class_type,name)
            # use the reasoner for the ontology
            reason()
    except rospy.ServiceException as e:
        print(e)      

##
#	\brief It checks if an hypothesis is complete and not inconsistent
#	\param ID : of string type, it is the hypothesis identifier 
#	\return : returns 1 if the hypothesis is complete and not inconsistent
#    returns 0 if it is either incomplete or inconsistent.
# 	
#	This function calls the armor server twice, the first time it retrieves all
#   the complete hypothesis and it checks if the searched hypothesis is in
#   that list, it then does the same to check if the hypothesis is inconsistent.
def check_complete_consistent(ID):
    #check if the ID hypothesis is completed
    try:
        completed=0
        inconsistent=0
        # set the request for the armor server check all the completed hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        # for all the elements retrieved I check if their ID is equal
        # to the one that needs to be checked
        for i in range(len(res_final)):
            if res_final[i]==ID:
                completed=1
        # set the request for the armor server check all the inconsistent hypothesis
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        # send the request
        msg = armor_service(req)
        # save the response of the server
        res=msg.armor_response.queried_objects
        # clean the results by removing usless parts
        res_final=clean_queries(res)
        # for all the elements retrieved I check if their ID is equal
        # to the one that needs to be checked
        for i in range(len(res_final)):
            if res_final[i]==ID:
                inconsistent=1
        # if the hypothesis is completed AND inconsistent return 1
        if completed==1 and inconsistent==0:
            return 1
        else :
            return 0
    except rospy.ServiceException as e:
        print(e)

if __name__ == '__main__':
  main()        
        
   
