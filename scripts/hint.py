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
    
def load_file(self):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'LOAD'
        req.primary_command_spec= 'FILE'
        req.secondary_command_spec= ''
        req.args= ['/root/ros_ws/src/exp_robotics_assignment1/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        #msg = self.armor_service(req)
    except rospy.ServiceException as e:
        print(e)
def initialize_person(self,person):
    try:
        req=ArmorDirectiveReq()
        req.client_name= 'tutorial'
        req.reference_name= 'ontoTest'
        req.command= 'ADD'
        req.primary_command_spec= 'IND'
        req.secondary_command_spec= 'CLASS'
        req.args= [person,'PERSON']
        self.reason()
    except rospy.ServiceException as e:
        print(e)


def main():
  armor_library=Armor_Communication()
  rospy.init_node('Init')
  load_file()
  
  rospy.spin() 

if __name__ == '__main__':
  main()        
        
   
