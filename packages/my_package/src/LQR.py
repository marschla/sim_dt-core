#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS,NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose
import numpy as np
import time
import control

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)
        # construct publisher
        self.pub_wheels_cmd = rospy.Publisher("~cmd", WheelsCmdStamped, queue_size=1)
        #self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        #subscriber
        self.sub_pose = rospy.Subscriber("~pose", LanePose, self.control, queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        self.omega = 0.0
        self.vref = 0.23    #v_ref defines speed at which the robot moves 
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

            #state feedback: statevector x = [d;phi]
            #K places poles at -1 and -2

            Ts = dt

            #computing integral state
            self.dint += self.dist*dt
            '''
            #discrete state space matrices (without integral d state)
            A = np.matrix([[1,self.vref*Ts],[0,1]])
            B = np.matrix([[0.5*Ts*Ts*self.vref],[Ts]])

            #weighting matrices
            Q = np.matrix([[2.5,0],[0,1]])
            R = np.matrix([[0.1]])
            '''
            
            #discrete state space matrices (with integral d state)
            A = np.matrix([[1,self.vref*Ts,0],[0,1,0],[Ts,0.5*self.vref*Ts*Ts,1]])
            B = np.matrix([[0.5*self.vref*Ts*Ts],[Ts],[self.vref*Ts*Ts*Ts/6.0]])

            Q = np.matrix([[4.5,0.0,0.0],[0.0,3.0,0.0],[0.0,0.0,0.5]])
            R = np.matrix([[0.05]])
            

            #solving dare
            (X,L,G) = control.dare(A,B,Q,R)

            #extracting K-matrix-components
            k1 = G[0,0]
            k2 = -G[0,1]
            k3 = G[0,2]

            # u = -K*x   (state feedback)
            #self.omega = -k1*self.dist - k2*self.phi
            self.omega  = -k1*self.dist - k2*self.phi -k3*self.dint


            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - self.L * self.omega
            car_cmd_msg.vel_right = self.vref + self.L * self.omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = k1
            message2 = k2
            message3 = self.omega


            rospy.loginfo('k1: %s' % message1)
            rospy.loginfo('k2: %s' % message2)
            rospy.loginfo('omega: %s' % message3)
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
    def control(self,pose):
        self.dist = -pose.d
        self.phi = pose.phi


    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
