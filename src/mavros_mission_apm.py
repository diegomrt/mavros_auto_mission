#!/usr/bin/env python

##########################################################################
# JdeRobot. APM + MAVROS: Load a mission using WPs file + fly fully AUTO
#
# Diego Martin
# 24/02/2019
# v1.0
###########################################################################

import rospy
import time
import os
import mavros
from mavros import command
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

mavros.set_namespace()

# Class to manage flight mode (APM stack).
class apmFlightMode:
    
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 2.5)
		rospy.loginfo("Taking off")
    	except rospy.ServiceException, e:
    		rospy.logerror("Takeoff failed: %s"%e)


    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='LAND')
            rospy.loginfo("Landing")
        except rospy.ServiceException, e:
	    rospy.logerror("Landing failed: %s. Land Mode could not be set"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            	armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            	armService(True)
	    	rospy.loginfo("Arming motors command sent OK") # It doens't mean the copter arms, just the command is sent properly. Add checking arming
        except rospy.ServiceException, e:
            	rospy.logerror("Arming motors command sending failed: %s"%e)

    def loadMission(self):
	    # Load mission from file (MP format). 
	    # To do: change to avoid using os.system + mavwp
                     # Handle errors pending!
	    os.system("rosrun mavros mavwp load ~/catkin_ws/src/mavros_auto_mission/missions/missionwp_file.txt") # Load new mission
	    rospy.loginfo("Mission WayPoints LOADED!")
	    os.system("rosrun mavros mavwp show") # Show mission WP loaded
	    

    def setAutoMissionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO')
	    rospy.loginfo("Entering AUTO mode OK")
        except rospy.ServiceException, e:
            rospy.logerror("Entering AUTO failed: %s. AUTO mode could not be set."%e)


    def read_failsafe(self):
        try:	    
	# Gets the current values of PX4 failsafe-related parameters 
	    get = rospy.ServiceProxy(mavros.get_topic('param', 'get'), ParamGet)
	    DLL_param = get(param_id="NAV_DLL_ACT") # Datalink Failsafe PX4 Parameter
	    RCL_param = get(param_id="NAV_RCL_ACT") # RC Failsafe PX4 Parameter    
	    print " "
	    print "----------PX4 FAILSAFE STATUS--------------"
	    print " Present NAV_DLL_ACT value is", DLL_param.value.integer
	    print " Present NAV_RCL_ACT value is", RCL_param.value.integer
	    print "-------------------------------------------"
	    print " "
	    return {'DL':DLL_param.value.integer,'RC':RCL_param.value.integer}
        except rospy.ServiceException, e:
            rospy.logerror("Failsafe Status read failed: %s"%e)

    def remove_failsafe(self):
        try:
	    # Disables both Datalink and RC failsafes  
	    val = ParamValue(integer=0, real=0.0) # Int value for disabling failsafe    
	    set = rospy.ServiceProxy(mavros.get_topic('param', 'set'), ParamSet)
	    new_DLL_param = set(param_id="NAV_DLL_ACT", value=val) 
	    new_RCL_param = set(param_id="NAV_RCL_ACT", value=val)    
	    print " "
	    print "----------REMOVING PX4 FAILSAFES ----------"
	    print " New NAV_DLL_ACT value is", new_DLL_param.value.integer
	    print " New NAV_RCL_ACT value is", new_RCL_param.value.integer
	    print "--------------------------------------------"
	    print " "
        except rospy.ServiceException, e:
            rospy.logerror("Failsafe Status change failed: %s"%e)


# WP reached Callback function. Controls spam by publishing once per WP!
def WP_callback(msg):
    
    global last_wp # Maybe there is a more elegant way of doint this :-)
    global starting_time
    global mission_started # Bool handles reaching WP#0 twice (mission beginning and end) 
    
    try: 
	mission_started
    except NameError: # Mission begins
	rospy.loginfo("Starting MISSION: Waypoint #0 reached")
	starting_time = msg.header.stamp.secs
	mission_started = True
    else: # Mission ongoing
        if msg.wp_seq == 0 and msg.wp_seq != last_wp: # Returning to WP #0 (last)
	   elapsed_time = msg.header.stamp.secs-starting_time	
	   rospy.loginfo("Ending MISSION: Total time: %d s", elapsed_time)
        elif msg.wp_seq != last_wp: # Intermediate WPs
	   elapsed_time = msg.header.stamp.secs-starting_time	
	   rospy.loginfo("MISSION Waypoint #%s reached. Elapsed time: %d s", msg.wp_seq, elapsed_time)
   
   # Update last WP reached
    last_wp=msg.wp_seq


# Main Function
def main():
   
    rospy.init_node('auto_mission_node', anonymous=True)

   # Flight mode object
    APMmodes = apmFlightMode()

   # Read Datalink and RC failsafe STATUS. Remove if present (for SITL)!
    failsafe_status = APMmodes.read_failsafe()
   #if (failsafe_status['DL'] != 0) or (failsafe_status['RC'] != 0):   
   #     APMmodes.remove_failsafe() 

   # AUTO MISSION: set mode, read WPs and Arm!  
    APMmodes.loadMission()
    APMmodes.setArm() # Previous to AUTO!
    APMmodes.setAutoMissionMode()
    
   # Subscribe to drone state to publish mission updates
    sub=rospy.Subscriber('mavros/mission/reached', WaypointReached, WP_callback)
   
   # Keep main loop
    rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass







