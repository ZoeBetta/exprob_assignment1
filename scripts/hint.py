import copy
import math
import sys
import time
from logging import setLoggerClass
from math import cos, pi, sin
from os import access
from re import X

import geometry_msgs.msg
import numpy as np
import rospy
from std_msgs.msg import String
from armor_msgs.msg import * 
from armor_msgs.srv import * 
from random import randint

people=[]
weapons=[]
locations=[]
armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)


def main():
  #armor_library=Armor_Communication()
  rospy.init_node('Init')
  rospy.wait_for_service('armor_interface_srv')

  sub_odom = rospy.Subscriber('/hint', String, clbk_hint)
  load_file()
  
  rospy.spin() 
    
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
        #res=ArmorDirectiveRes()
        res=msg.armor_response
        print(res)
    except rospy.ServiceException as e:
        print(e)
        
def initialize_person(person):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [person,'PERSON']
        msg = armor_service(req)
        #res=ArmorDirectiveRes()
        res=msg.armor_response
        #print(res)
        #print(person)
    except rospy.ServiceException as e:
        print(e)
        
def initialize_weapon(weapon):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [weapon,'WEAPON']
        msg = armor_service(req)
    except rospy.ServiceException as e:
        print(e)
        
def initialize_location(location):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [location,'LOCATION']
        msg = armor_service(req)
    except rospy.ServiceException as e:
        print(e)
        
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
    
def disjoint(what):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'DISJOINT'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [what]
        msg = armor_service(req)		 
    except rospy.ServiceException as e:
        print(e)        
 
def print_people():
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'QUERY'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= ['PERSON']
        msg = armor_service(req)
        #res=ArmorDirectiveRes()
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        print(res_final)
    except rospy.ServiceException as e:
        print(e)	

def clean_queries(query):
    for i in range(len(query)):
        temp=query[i]
        temp=temp.split('#')
        index=len(temp)
        temp=temp[index-1]
        query[i]=temp[:-1]
    return query
  
def check_if_received_before(data):
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
        

def clbk_hint(msg):
	
	s=str(msg.data)
	hint_received=s.split('/')
	find=check_if_received_before(hint_received)
	if find==0:
	    if hint_received[1]=='who':
		    print(hint_received[2])
		    initialize_person(hint_received[2])
		    reason()
		    disjoint('PERSON')
		    reason()
		    print_people()
	    if hint_received[1]=='what':
		    initialize_weapon(hint_received[2])
		    reason()
		    disjoint('WEAPON')
		    reason()
	    if hint_received[1]=='where':
		    initialize_location(hint_received[2])
		    reason()
		    disjoint('LOCATION')
		    reason()	    
	    print('added')
	if find==1:
		print('already there')
    
	# check if already present in the ontology as a hypothesis
	    # if yes stop
	    # if not add an hypothesis
	        # check if complete
	        # if complete check if not inconsistent
	            # if yes- send on the topic hypothesis the ID and the three strings separate
	            #if no stop
    
	
	



if __name__ == '__main__':
  main()        
        
   
