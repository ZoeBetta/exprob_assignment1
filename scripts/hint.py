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
  armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
  pub=rospy.Publisher('/hypothesis', Hypothesis, queue_size=10)
  rospy.wait_for_service('armor_interface_srv')
  sub_odom = rospy.Subscriber('/hint', String, clbk_hint)

  load_file()
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
    s=str(msg.data)
    hint_received=s.split('/')
    rospy.set_param('ID', hint_received[0])
    print(rospy.get_param('ID'))
    find=check_if_received_before(hint_received)
    if find==0:
        add_instance(hint_received[2],hint_received[1])	    
    check_in_ontology(hint_received[0], hint_received[1], hint_received[2])
    send=check_complete_consistent(hint_received[0])
    if send==1:
        for i in range(len(hypothesis)):
            if hint_received[0]==hypothesis[i]:
                already_done=1
        if already_done==0:
            print('send to robot')
            hypothesis.append(hint_received[0])
            message= Hypothesis()
            message.ID=hint_received[0]
            temp=look_hypothesis(hint_received[0], 'who')
            message.who=temp[0]
            temp=look_hypothesis(hint_received[0], 'what')
            message.what=temp[0]
            temp=look_hypothesis(hint_received[0], 'where')
            message.where=temp[0]
            print(message)
            pub.publish(message)
    else:
        print (' not complete or not consistent')

##
#	\brief This function loads the ontology
#	\param : None 
#	\return : None
# 	
#	It loads the ontology from a file called cluedo_ontology in a specific
#   path. Be careful if you want to change the path of the file it needs
#   to be done here. The file is loaded by the proper call to the armor server.
def load_file():
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''
        req.args= ['/root/ros_ws/src/exprob_assignment1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
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
        class_id=find_type(class_type)
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [name, class_id]
        msg = armor_service(req)
        res=msg.armor_response
        reason()
        disjoint(class_id)
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
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'REASON'
        req.primary_command_spec= ''
        req.secondary_command_spec= ''
        req.args= []
        msg = armor_service(req)
        res=msg.armor_response
        #print(res)
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
def disjoint(class_type):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [class_type]
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
#   the last element of the remaing string.
def clean_queries(query):
    for i in range(len(query)):
        temp=query[i]
        temp=temp.split('#')
        index=len(temp)
        temp=temp[index-1]
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
    if data[1]=='who':
        for i in range(len(people)):
            if people[i]==data[2]:
                find=1;
        if find==0:
            people.append(data[2])
    if data[1]=='what':
        for j in range(len(weapons)):
            if weapons[j]==data[2]:
                find=1;
        if find==0:
            weapons.append(data[2])
    if data[1]=='where':
        for k in range(len(locations)):
            if locations[k]==data[2]:
                find=1;
        if find==0:
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
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID,name]
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
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'OBJECTPROP'
        req.secondary_command_spec= 'IND'
        req.args= [class_type,ID]
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
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
        res_final=look_hypothesis(ID, class_type)
        namef.append(name)
        if res_final != namef:
            add_hypothesis(ID,class_type,name)
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
    #check completed
    try:
        completed=0
        inconsistent=0
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['COMPLETED']
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        for i in range(len(res_final)):
            if res_final[i]==ID:
                completed=1
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['INCONSISTENT']
        msg = armor_service(req)
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        for i in range(len(res_final)):
            if res_final[i]==ID:
                inconsistent=1
        if completed==1 and inconsistent==0:
            return 1
        else :
            return 0
    except rospy.ServiceException as e:
        print(e)

if __name__ == '__main__':
  main()        
        
   
