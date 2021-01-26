#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose
import numpy as np
import time

class ControllerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ControllerNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        # construct publisher
        self.pub_wheels_cmd = rospy.Publisher("~cmd", WheelsCmdStamped, queue_size=1,dt_topic_type=TopicType.CONTROL)
        #self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        
        #subscriber
        self.sub_pose = rospy.Subscriber("~pose", LanePose, self.control, "lane_filter", queue_size=1)
        
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        
        #def. variables
        self.vdiff = 0.0
        self.omega = 0.0
        self.vref = 0.23    #v_ref defines speed at which the robot moves 
        self.dist = 0.0
        self.dold = 0.0
        self.phiist = 0.0
        #self.phiref = 0.0
        self.phiest = 0.0
        self.C_i = 0

        self.mtime = 0.0
        self.mtimeold = 0.0

        self.time_arr = []

        #structural paramters of duckiebot
        self.L = 0.05      #length from point A to wheels [m]


    #control alg. for inner loop
    def getomega(self,phiref,phiist,dt):

        #PID params for inner loop
        k_p = 3.8
        k_i = 0.1
        k_d = 0.0
        sati = 1.0
        satd = 1.0
        omegasat = 100.0
    
        err = self.phiest-phiref

        C_p = k_p*err

        self.C_i += k_i*dt*err

        omega = C_p + self.C_i

        if omega>omegasat:
            omega=omegasat
        if omega<-omegasat:
            omega=-omegasat

        self.phiest += omega*dt

        return omega

    def getphiref(self,dist):

        #PID params for outer loop
        k_p = 3.0
        k_i = 0.25
        k_d = 0.0
        sat = np.pi/4.0

        #correct sign for controller, (offset because DB drove too far on the right)
        err = -dist

        phiref = k_p*err

        #saturation, currently at pi/2 (needs testing if bigger angles needed (especially in tight turns))
        if phiref>sat:
            phiref=sat
        if phiref<-sat:
            phiref=-sat

        return phiref

    def run(self):
        # publish message every 1/x second
        #for cascade choose rate > rate of camera    (rate of camera 8-35 Hz)
        rate = rospy.Rate(50) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        t0=tnew
        stoptime = 100.0
        while not rospy.is_shutdown():
            #computing dt for I and D-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told

            phiref = self.getphiref(self.dist)
            self.omega = self.getomega(phiref,self.phiist,dt)
            

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - self.L * self.omega
            car_cmd_msg.vel_right = self.vref + self.L * self.omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = self.omega
            message3 = self.phiist
            message4 = phiref
            message5 = tnew-t0

            #rospy.loginfo('d: %s' % message1)
            #rospy.loginfo('phi: %s' % message3)
            #rospy.loginfo('phiref: %s' % message4)
            rospy.loginfo("omega = %s" % message2)

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
    def control(self,pose, source):
        self.dist = pose.d
        self.phiist = pose.phi
        #updating our estimate of phi
        self.phiest = pose.phi

    
if __name__ == '__main__':
    # create the node
    lane_controller_node = ControllerNode(node_name='lane_controller_node')
    # run node
    lane_controller_node.run()
    # keep spinning

    #node.save()

    rospy.spin()
