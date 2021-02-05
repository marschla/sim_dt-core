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

        # Publisher
        #Publishes actuator commands to node handling the wheel commands
        self.pub_wheels_cmd = rospy.Publisher("~cmd", WheelsCmdStamped, queue_size=1,dt_topic_type=TopicType.CONTROL)
        #self.pub_omega = self.publisher(str(os.environ['VEHICLE_NAME'])+"/kinematics_node/velocity", Twist2DStamped, queue_size=1)
        
        # Subscriber
        #Subscribes to the node publishing the LanePose estimation
        self.sub_pose = rospy.Subscriber("~pose", LanePose, self.control, "lane_filter", queue_size=1)
        
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)
        
        #def. variables
        self.vref = rospy.get_param("~vref",None)    #v_ref defines speed at which the robot moves 
        self.dist = 0.0     #class variable to store distance do lanecenter
        self.phiist = 0.0   #class variable to store last estimate of phi from lanefilter
        self.phiest = 0.0   #class variable to store current estimate of heading
        self.C_i = 0        #variable to keep track of integral state

        #structural paramters of duckiebot
        self.baseline = rospy.get_param('~baseline', None)      #distance between the two wheels

        #parameters for inner loop
        self.k_p_i = rospy.get_param('~k_p_i', None)
        self.k_i_i = rospy.get_param('~k_i_i', None)
        self.k_d_i = rospy.get_param('~k_d_i', None)
        self.sati_i = rospy.get_param('~sati_i', None)
        self.satd_i = rospy.get_param('~satd_i', None)
        self.omegasat = rospy.get_param('~omegasat', None)

        #parameters for outer loop
        self.k_p_o = rospy.get_param('~k_p_o', None)
        self.k_i_o = rospy.get_param('~k_i_o', None)
        self.k_d_o = rospy.get_param('~k_d_o', None)
        self.sat_phi = rospy.get_param('~sat_phi', None)


    #control alg. for inner loop
    #Using the angular offset and computed referene angle, this function returns the control action (ie omega)
    def getomega(self,phiref,dt):

        #errror term for inner loop
        err = self.phiest-phiref
        #proportional term
        C_p = self.k_p_i*err
        #integral term
        self.C_i += self.k_i_i*dt*err
        #control action
        omega = C_p + self.C_i

        #saturation of omega -> making sure it does not become too big
        if omega>self.omegasat:
            omega=self.omegasat
        if omega<-self.omegasat:
            omega=-self.omegasat

        #Since our innerloop should have a higher freq then the data comming in from LanePose estimation
        #we update the heading measurement
        self.phiest += omega*dt

        return omega

    #outer loop
    #using lateral offset, this function returns reference heading for inner loop
    def getphiref(self,dist):

        #correct sign for controller, because of sign conventions
        err = -dist

        phiref = self.k_p_o*err

        #saturation, currently at pi/2 (needs testing if bigger angles needed (especially in tight turns))
        #because too low saturation can lead to too slow reaction to Turns
        if phiref>self.sat_phi:
            phiref=self.sat_phi
        if phiref<-self.sat_phi:
            phiref=-self.sat_phi

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

            #running controller functions for inner and outer loop
            #Note: here both functions run at the same frequency, but since d is only updated once a new Poseestimation
            #is received the output of getphiref stays constant, while omega changes, because we update phiest using our
            #computed controlaction (but we do not update d)
            phiref = self.getphiref(self.dist)
            omega = self.getomega(phiref,dt)
            

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - 0.5*self.baseline * omega
            car_cmd_msg.vel_right = self.vref + 0.5*self.baseline * omega

            self.pub_wheels_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = omega
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