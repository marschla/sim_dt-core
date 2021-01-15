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
        self.pub_car_cmd = rospy.Publisher("fakebot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        #Subscriber
        self.sub_lane_reading = rospy.Subscriber("fakebot/sim_node/lane_pose", LanePose, self.control, "lane_filter", queue_size=1)
        
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)

        #sys params
        self.omega = 0.0
        self.vref = 0.23    #v_ref defines speed at which the robot moves 
        self.dist = 0.0
        self.tist = 0.0

        #params used for PID control 
        self.C_id = 0.0
        self.C_iphi = 0.0

        self.L = 0.05

        self.stamp = 0
        self.header = 0

    def resetintegral(self,d,phi):
        tol_d = 0.05
        tol_phi = 0.1

        if np.abs(d) <= tol_d and np.abs(phi) <= tol_phi:
            self.C_i = 0
            rospy.loginfo("Reset Integral")

    def getomega(self,dist,phi,dt):
        #parameters for PID control
        k_dd = 6.0
        k_id = 0.5
        k_dphi = 5.0
        k_iphi = 0.5
        sat_id = 1.0
        sat_iphi = 1.0
        omegasat = 5.0

        self.C_id += dist*dt
        self.C_iphi += phi*dt

        if self.C_id > sat_id:
            self.C_id = sat_id
        if self.C_id < -sat_id:
            self.C_id = -sat_id

        if self.C_iphi > sat_iphi:
            self.C_iphi = sat_iphi
        if self.C_iphi < -sat_iphi:
            self.C_iphi = -sat_iphi

        omega = k_dd*dist + k_dphi*phi + k_id*self.C_id + k_iphi*self.C_iphi
        
        if omega>omegasat:
            omega=omegasat
        if omega<-omegasat:
            omega=-omegasat
        
        return omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        t0 = time.time()
        i=0
        
        while  not rospy.is_shutdown():
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told
            
            #self.vdiff = self.getvdiff(self.dist,self.tist,dt)
            self.omega = self.getomega(self.dist,self.tist,dt)

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - self.L * self.omega
            car_cmd_msg.vel_right = self.vref + self.L * self.omega

            self.pub_car_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = self.omega
            message3 = self.tist
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
        self.tist = pose.phi
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