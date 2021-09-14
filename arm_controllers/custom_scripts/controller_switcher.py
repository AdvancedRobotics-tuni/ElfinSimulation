#!/usr/bin/env python

from tf.transformations import *
import rospy
import sys
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, LoadController, LoadControllerRequest



"""
switching controllers using SwitchController service
the server is already implemented in ros, we just need client here to send request
"""

def switch_controller(switch_control_client, controller_number):

    rospy.loginfo('Switching controller')
    switch_msg = SwitchControllerRequest()
    
    if controller_number == 1:
    	switch_msg.start_controllers = ["cloned_clik_controller_1","joint_state_controller"]
    	switch_msg.stop_controllers = ["cloned_gravity","joint_state_controller"]
    	switch_msg.strictness = 1
    if controller_number == 2:
    	switch_msg.start_controllers = ["cloned_gravity","joint_state_controller"]
    	switch_msg.stop_controllers = ["cloned_clik_controller_1","joint_state_controller"]
    	switch_msg.strictness = 1
    switch = switch_control_client(switch_msg)
    

    return switch.ok
       

"""
it is needed to load your controller before starting it, otherwise it fails
the function uses ros control services (LoadController service)
"""

def load_controller(load_control_client, name):
    rospy.loginfo('loading controller')
    load_msg = LoadControllerRequest()
    load_msg.name = name
    return load_control_client(load_msg).ok
    
"""
a callback function that only listen to /rqt_command_listener topic and return its values to the listener of 
controller node
"""  
def callback(data):
	
    global pub
    msg = Float64MultiArray()
    msg.data = [data.data[0],data.data[1],data.data[2],0,0,0]
    pub.publish(msg)
    
	
if __name__ == "__main__":

    """
    initialize ros node and create publisher and subscriber
    """
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/elfin/cloned_clik_controller_1/command', Float64MultiArray, queue_size=1)
    sub = rospy.Subscriber("/rqt_command_listener", Float64MultiArray, callback)

    """
    waiting for the services to load before using it
    """
    print ("waiting for the service")
    rospy.wait_for_service('/elfin/controller_manager/switch_controller')
    rospy.wait_for_service('/elfin/controller_manager/load_controller')
    print ("service loaded")
    
    
    switch_control_client = rospy.ServiceProxy('/elfin/controller_manager/switch_controller', SwitchController)
    load_control_client = rospy.ServiceProxy('/elfin/controller_manager/load_controller', LoadController)
    
    """
    loading the controllers that are not already loaded with launch file
    """
    print("loading controller")
    print (load_controller(load_control_client, "cloned_gravity"))
   
    print("write controller number and press enter")
    controller = input()

    print (switch_controller(switch_control_client, int(controller)))
    
    rospy.spin()
    
    

