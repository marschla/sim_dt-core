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
        self.pub_car_cmd = rospy.Publisher("~cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        #Subscriber
        self.sub_lane_reading = rospy.Subscriber("~pose", LanePose, self.control, "lane_filter", queue_size=1)
        
        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)

        #sys params
        self.vref = rospy.get_param("~vref",None)    #v_ref defines speed at which the robot moves 
        self.dist = 0.0     #class variable to store distance do lanecenter
        self.phi = 0.0      #class variable to store current estimate of heading

        #params used for PID control 
        self.C_i = 0.0       #variable to keep track of integral state

        #structural paramters of duckiebot
        self.baseline = rospy.get_param('~baseline', None)     #distance between the two wheels

        #Parameters for PID control
        self.k_p = rospy.get_param('~k_p', None)
        self.k_i = rospy.get_param('~k_i', None)
        self.k_d = rospy.get_param('~k_d', None)
        self.sati = rospy.get_param('~sati', None)
        self.satd = rospy.get_param('~satd', None)
        self.omegasat = rospy.get_param('~omegasat', None)


    #function to reset Integral term, if robot is driving perfectly in lane (+- some tolerance)
    def resetintegral(self,d,phi):
        tol_d = 0.05
        tol_phi = 0.1

        if np.abs(d) <= tol_d and np.abs(phi) <= tol_phi:
            self.C_i = 0
            rospy.loginfo("Reset Integral")

    #Using the Laneposeestimation, this function computes controlaction
    def getomega(self,dist,phi,dt):
        
        #defining error term for PID contrller
        err = phi

        #proportional gain part
        C_p = self.k_p*err

        #integral term (approximate integral)
        self.C_i += self.k_i*dt*err

        #uncomment, if integral reset should be allowed
        #self.resetintegral(dist,tist)
        
        #make sure integral term doesnt become too big
        if self.C_i > self.sati:
            self.C_i = self.sati 
        if self.C_i < -self.sati:
            self.C_i = -self.sati 
        
        
        #computing control output
        omega = C_p + self.C_i 
        
        #saturation of omega -> making sure it does not become too big
        if omega>self.omegasat:
            omega=self.omegasat
        if omega<-self.omegasat:
            omega=-self.omegasat


        #computed velocity
        #v = np.sqrt(1.0-2*np.abs(dist)+2*(dist)**2)*self.vref
        v=self.vref
        
        return v,omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        tnew = time.time()
        stoptime = 28.0
        t0 = time.time()
        i=0
        
        while  not rospy.is_shutdown():
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told

            #call function to compute controlaction
            v,omega = self.getomega(self.dist,self.phi,dt)

            #car_cmd_msg.omega = self.omega
            #car_cmd_msg.v = self.vref
            #car_cmd_msg.header = self.header

            #print warning, if controloutput reaches satureation
            if np.abs(omega) >= 5.0:
                rospy.logwarn("Max Omega reached")

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = v - 0.5*self.baseline * omega
            car_cmd_msg.vel_right = v + 0.5*self.baseline * omega

            self.pub_car_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = omega
            message3 = v
            message4 = dt

            #rospy.loginfo('d: %s' % message1)
            rospy.loginfo('omega: %s' % message2)
            rospy.loginfo('v: %s' % message3)
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
     

if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()