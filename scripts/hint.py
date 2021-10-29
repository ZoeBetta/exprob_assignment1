#! usr/bin/env python2


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

people=[]
weapons=[]
locations=[]
hypothesis=[]
armor_service = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
pub=rospy.Publisher('/hypothesis', Hypothesis, queue_size=10)


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
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)
        
def initialize(person, class_type):
    try:
        class_id=find_type(class_type)
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [person, class_id]
        msg = armor_service(req)
        res=msg.armor_response
        reason()
        disjoint(class_id)
        reason()
    except rospy.ServiceException as e:
        print(e)

def find_type(class_type):
    if class_type=='who':
        return 'PERSON'
    if class_type== 'what':
        return 'WEAPON'
    if class_type=='where':
        return 'LOCATION' 
        
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
        res=msg.armor_response.queried_objects
        res_final=clean_queries(res)
        #print(res_final)
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
        #print(res)
    except rospy.ServiceException as e:
        print(e)	

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

def check_in_ontology(ID,class_type,name):
    try:
        namef=[]
        res_final=look_hypothesis(ID, class_type)
        namef.append(name)
        if res_final != namef:
            #print('diverso aggiungi')
            add_hypothesis(ID,class_type,name)
            reason()
    except rospy.ServiceException as e:
        print(e)      

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
                if inconsistent==1:
                    print('INCONSISTENT')  
        if completed==1 and inconsistent==0:
            return 1
        else :
            return 0
    except rospy.ServiceException as e:
        print(e)


def clbk_hint(msg):
    already_done=0
    s=str(msg.data)
    hint_received=s.split('/')
    rospy.set_param('ID', hint_received[0])
    print(rospy.get_param('ID'))
    find=check_if_received_before(hint_received)
    if find==0:
        initialize(hint_received[2],hint_received[1])	    
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

if __name__ == '__main__':
  main()        
        
   
