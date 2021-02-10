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
        self.dist = 0.0     #class variable to store distance do lanecenter
        self.phi = 0.0      #class variable to store last estimate of phi from lanefilter
        self.baseline = rospy.get_param('~baseline', None)    #distance between the two wheels

        #tuning parameters
        self.L = rospy.get_param('~L', None)               #offset param, at which point ahead of A the LanePose is applied to
        self.lookahead = rospy.get_param('~lookahead', None)    #lookahead distance to obtain phiref
        self.vref = rospy.get_param('~vref', None)
        self.omegasat = rospy.get_param('~omegasat', None)
        self.phirefsat = rospy.get_param('~phirefsat', None)
        self.tol = rospy.get_param('~tolerance', None)       #tolerance param, of DB is closer to the centerline as the tolerance, omega as automatically set to zero

    #using LanePose estimation, this function computes controlaction
    def getomega(self,dist,phi):     
        
        #computing phiref according to theory
        phiref = np.arctan2(dist,self.lookahead)

        #saturation for phiref
        if phiref > self.phirefsat:
            phiref = self.phirefsat
        if phiref < -self.phirefsat:
            phiref = -self.phirefsat

        '''
        #outdated
        #This part sets omega to zero, if DB is in a certain bound close to the centerline
        if np.abs(dist) > self.tol:
            omega = self.vref / self.L * np.sin(phiref - phi)
            v = self.vref * np.cos(phiref - phi)
        else:
            omega = 0.0 
            v  = self.vref
        '''

        rospy.loginfo("phiref = %s" % phiref)


        #computing control action following the GotoAngle control architecture (see Thesis for more information)
        if phiref - phi > np.pi/2.0:
            omega = self.omegasat
            v=0.0
        elif phiref - phi < -np.pi/2.0:
            omega = -self.omegasat
            v=0.0
        else:
            omega = self.vref / self.L * np.sin(phiref - phi)
            v = self.vref * np.cos(phiref - phi)
        
        #saturation of omega -> making sure it does not become too big
        if omega>self.omegasat:
            omega=self.omegasat
        if omega<-self.omegasat:
            omega=-self.omegasat
        
        return v,omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()

        
        while  not rospy.is_shutdown():

            #compute control action
            v,omega = self.getomega(-self.dist,self.phi)

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = v + 0.5*self.baseline * omega
            car_cmd_msg.vel_right = v - 0.5*self.baseline * omega

            self.pub_car_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = self.phi
            message3 = omega
            message4 = v

            rospy.loginfo('d: %s' % message1)
            rospy.loginfo('phi: %s' % message2)
            rospy.loginfo('omega: %s' % message3)
            rospy.loginfo('v: %s' % message4)
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
        self.dist = pose.d
        self.phi = pose.phi
        #delay = rospy.Time.now() - pose.header.stamp
        #delay_float = delay.secs + float(delay.nsecs)/1e9    
        #rospy.loginfo('delay [s] =  %s' % delay_float)       

if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()