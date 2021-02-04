#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose
import numpy as np
import time

class ControllerNode(DTROS):

    def __init__(self,node_name):
        # Initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        #Publisher
        #Publishes actuator commands to node handling the wheel commands
        self.pub_car_cmd = rospy.Publisher("~cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        #Subscriber
        #Subscribes to the node publishing the LanePose estimation
        self.sub_lane_reading = rospy.Subscriber("~pose", LanePose, self.control, "lane_filter", queue_size=1)
        
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)

        #sys params
        self.vref = 0.23    #v_ref defines speed at which the robot moves 
        self.dist = 0.0     #class variable to store distance do lanecenter
        self.phi = 0.0     #class variable to store current estimate of heading

        #structural paramters of duckiebot
        self.baseline = 0.1     #distance between the two wheels

        self.stamp = 0
        self.header = 0

    #compute controlaction based on Stanley Controller theory
    def getomega(self,dist,phi):
        #parameters for Stanley control, k_phi sould always be bigger then 1, -> see thesis for more information
        omegasat = 4.5
        k_d = 10.0
        k_phi = 5.0
        
        rospy.loginfo("phi = %s" % phi)
        rospy.loginfo("arctan = %s" % np.arctan2(k_d*dist,self.vref))

        omega = k_phi*phi + np.arctan2(k_d*dist,self.vref)
        
        #saturation of omega -> making sure it does not become too big
        if omega>omegasat:
            omega=omegasat
        if omega<-omegasat:
            omega=-omegasat
        
        return omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        i=0
        
        while  not rospy.is_shutdown():

            #call function to compute controlaction
            omega = self.getomega(self.dist,self.phi)

            #car_cmd_msg.omega = self.omega
            #car_cmd_msg.v = self.vref
            #car_cmd_msg.header = self.header

            #print warning, if omega reaches saturated state
            if np.abs(self.omega) >= 4.5:
                rospy.logwarn("Max Omega reached")

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - 0.5*self.baseline * omega
            car_cmd_msg.vel_right = self.vref + 0.5*self.baseline * omega

            self.pub_car_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = omega
            message3 = self.phi
            message4 = dt

            #rospy.loginfo('d: %s' % message1)
            rospy.loginfo('omega: %s' % message2)
            #rospy.loginfo('phi: %s' % message3)
            #rospy.loginfo('dt: %s' % message4)
            #rospy.loginfo("time: %s" % message5)
            
            rate.sleep()

    #shutdown procedure, stopping motor movement etc.
    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_car_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?????")

    #function updates pose variables, that camera gives us data at higher rate then this code operates at,
    #thus we do not use all incoming data
    def control(self,pose, source):
        self.header = pose.header
        self.dist = pose.d
        self.phi = pose.phi    

if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()