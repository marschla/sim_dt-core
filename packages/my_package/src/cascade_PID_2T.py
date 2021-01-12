#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS,NodeType
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
        self.sub_pose = rospy.Subscriber("fakebot/sim_node/lane_pose", LanePose, self.control, queue_size=1)
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        #def. variables
        self.omega = 0.0
        #self.vref = 0.23    #v_ref defines speed at which the robot moves 
        self.dist = 0.0
        self.dold = 0.0
        self.phi = 0.0

        #params used for PID control 
        self.C_p = 0.0
        self.C_i = 0.0
        self.C_d = 0.0
        self.arr_d = np.array([0.0,0.0,0.0])

        #structural paramters of duckiebot
        self.L = 0.05      #length from point A to wheels [m]


    def getphiref(self,dist,phi):
        L = 0.2        #lookahead parameter
        vmax = 0.23     #maximal velocity
        phisat = np.pi/4.0
        l = 0.05


        #we introduce this term, because the point we control and the point where the pose applies to are not the same
        dist_ = dist - l*np.cos(phi)        #minus sign arises from sign convention from lane pose

        phiref = np.arctan2(np.abs(dist_),L)

        if dist>0:
            phiref = -phiref
        if dist<0:
            phiref = phiref

        vref = vmax
                                                              
        #phiref saturating behavior
        if phiref>phisat:
            phiref=phisat
        if phiref<-phisat:
            phiref=-phisat

        return phiref,vref

    def getomega(self,err,dt):
        #parameters for PID control
        k_p = 2.5
        k_i = 0.75
        k_d = 0.0
        #saturation params
        sati = 1.0
        satd = 2.0
        omegasat=4.5

        #proportional gain part
        self.C_p = k_p*err

        #integral term (approximate integral)
        self.C_i += k_i*dt*err
        
        #make sure integral term doesnt become too big
        if self.C_i > sati:
            self.C_i = sati
        if self.C_i < -sati:
            self.C_i = -sati
        
        #derivative term
        self.C_d = k_d*(err-self.dold)/dt
        self.dold = err
        
        #make sure derivative term doesnt become too big
        if self.C_d > satd:
            self.C_d = satd
        if self.C_d < -satd:
            self.C_d = -satd
        
        #computing control output
        omega = self.C_p + self.C_i + self.C_d
        
        
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
        while not rospy.is_shutdown():
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told

            phiref, vref = self.getphiref(self.dist,self.phi)

            err = phiref - self.phi

            self.omega = self.getomega(err,dt)

            

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = vref + self.L * self.omega
            car_cmd_msg.vel_right = vref - self.L * self.omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = self.omega
            message3 = phiref
            message4 = vref

            rospy.loginfo('d: %s' % message1)
            rospy.loginfo('phiref: %s' % message3)
            rospy.loginfo('vref: %s' % message4)
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
    def control(self,pose):
        self.dist = pose.d
        self.phi = pose.phi
        #message = pose.d
        #rospy.loginfo('d =  %s' % message)

    
if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
