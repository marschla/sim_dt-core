#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose
import numpy as np
import time

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)
        # construct publisher
        #Publishes actuator commands to node handling the wheel commands
        self.pub_wheels_cmd = rospy.Publisher("~cmd", WheelsCmdStamped, queue_size=1)
        #self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        #subscriber
        #Subscribes to the node publishing the LanePose estimation
        self.sub_lane_reading = rospy.Subscriber("~pose", LanePose, self.control, "lane_filter", queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        #self.vref = rospy.get_param('~vref', None)    #vref defines speed at which the robot moves 
        self.vref = float(os.environ['SPEED'])
        self.dist = 0.0     #class variable to store distance do lanecenter
        self.phi = 0.0      #class variable to store current estimate of heading
        self.dint = 0.0     #class variable for integral state (of distance to lane center)


        #structural paramters of duckiebot
        self.baseline = rospy.get_param('~baseline', None)    #distance between the two wheels

        #defining, weather integral state should be considered (True - yes ; False - no)
        self.integral_state = rospy.get_param('~integral_state_enable', None)

        #Controller Parameter
        self.k1_n = rospy.get_param('~k1_n', None)
        self.k2_n = rospy.get_param('~k2_n', None)
        self.k1_i = rospy.get_param('~k1_i', None)
        self.k2_i = rospy.get_param('~k2_i', None)
        self.k3_i = rospy.get_param('~k3_i', None)
        self.sati = rospy.get_param('~sati', None)
        self.omegasat = rospy.get_param('~omegasat', None)
    
    #computes control actions with regard to LanePose estimation
    def getomega(self,dist,phi,dt):

        if self.integral_state == True:

            #with integral state
            #state feedback: statevector = [d;phi;dint]
            #current best K = [7.0 -3.25 1.1]

            #computing integral state
            self.dint += dt*dist
            #integral saturation, making sure, that it doesn't frow too big
            if self.dint > self.sati:
                self.dint = self.sati
            if self.dint < -self.sati:
                self.dint = -self.sati

            # u = -Kx
            omega = -self.k1_i*dist - self.k2_i*phi - self.k3_i*self.dint


        else:

            #without integral state
            #state feedback: statevector x = [d;phi]
            #K places poles at -1 and -2   (in continuous time)

            # u = -Kx
            omega = -self.k1_n*dist - self.k2_n*phi

        #output saturation 
        #making sure, that too large of actuator output is requested, 
        if omega > self.omegasat:
            omega = self.omegasat
        if omega < -self.omegasat:
            omega = -self.omegasat

        #keep velocity constant
        v = self.vref

        return v,omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        while not rospy.is_shutdown():
            #computing dt for integral state
            told = tnew
            tnew = time.time()
            dt = tnew-told

            #call function to compute controlaction
            v,omega = self.getomega(self.dist,self.phi,dt)
            
            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = v + 0.5*self.baseline * omega
            car_cmd_msg.vel_right = v - 0.5*self.baseline * omega

            self.pub_wheels_cmd.publish(car_cmd_msg)


            rospy.loginfo('d: %s' % self.dist)
            rospy.loginfo('phi: %s' % self.phi)
            rospy.loginfo('omega: %s' % omega)
            rate.sleep()

     #shutdown procedure, stopping motor movement etc.
    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_wheels_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")

    #function updates pose variables, that camera gives us data at higher rate then this code operates at,
    #thus we do not use all incoming data
    def control(self,pose,source):
        self.dist = pose.d
        self.phi = pose.phi


    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
