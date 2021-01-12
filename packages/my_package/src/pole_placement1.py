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
        self.pub_wheels_cmd = rospy.Publisher("fakebot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        #self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        #subscriber
        self.sub_lane_reading = rospy.Subscriber("fakebot/sim_node/lane_pose", LanePose, self.control, "lane_filter", queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        self.omega = 0.0
        self.vref = 0.23    #vref defines speed at which the robot moves 
        self.dist = 0.0
        self.phi = 0.0
        self.dint = 0.0


        #structural paramters of duckiebot
        self.L = 0.05      #length from point A to wheels [m]


    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        while not rospy.is_shutdown():
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told
            omegamax = 10.0
            sati = 1.0

            '''
            #without integral state
            #state feedback: statevector x = [d;phi]
            #K places poles at -1 and -2   (in continuous time)
            
            k1 = 10.5
            k2 = -4.25

            # u = -K*x
            self.omega = -k1*self.dist - k2*self.phi
            '''
            
            #with integral state
            #state feedback: statevector = [d;phi;dint]
            #current best K = [7.0 -3.25 1.1]

            self.dint += dt*self.dist

            if self.dint > sati:
                self.dint = sati
            if self.dint < -sati:
                self.dint = -sati

            k1 = 7.0
            k2 = -3.25
            k3 = 1.1

            self.omega = -k1*self.dist - k2*self.phi - k3*self.dint
            

            if self.omega > omegamax:
                self.omega = omegamax
            if self.omega < -omegamax:
                self.omega = -omegamax
            
            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - self.L * self.omega
            car_cmd_msg.vel_right = self.vref + self.L * self.omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lane_pose
            message1 = self.dist*k1
            message2 = self.omega
            message3 = self.phi*k2
            message4 = self.dint*k3


            rospy.loginfo('d: %s' % message1)
            rospy.loginfo('phi: %s' % message3)
            rospy.loginfo('dint: %s' % message4)
            rospy.loginfo('omega: %s' % message2)
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
        self.dist = -pose.d
        self.phi = pose.phi


    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
