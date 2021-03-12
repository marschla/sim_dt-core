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
        #self.pub_car_cmd = rospy.Publisher("fakebot/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)

        #Subscriber
        #Subscribes to the node publishing the LanePose estimation
        self.sub_lane_reading = rospy.Subscriber("~pose", LanePose, self.control, "lane_filter", queue_size=1)
        #self.sub_lane_reading = rospy.Subscriber("fakebot/lane_filter_node/lane_pose", LanePose, self.control, "lane_filter", queue_size=1)

        #shutdown procedure
        rospy.on_shutdown(self.custom_shutdown)

        #sys params
        self.omega_old = 0.0   
        self.dist = 0.0     #class variable to store distance do lanecenter
        self.dold = 0.0     #stores error from previous timestep for derivative term
        self.phi = 0.0     #class variable to store current estimate of heading

        #params used for PID control 
        self.C_i = 0.0      #class variable, to store integralstate

        #loading params from yaml file

        #self.baseline = 0.1     #distance between the two wheels
        self.baseline = rospy.get_param('~baseline', None)  #distance between the two wheels

        #weighting parameters should chosen such that weight_d + weight_phi = 1
        #and weight_d/weight_phi < pi/(2*d_max), where d_max should approx be the lanewidth*safetycoefficient (i.e. 1.2*lanewidth)
        self.weight_d = rospy.get_param('~weight_d', None)
        self.weight_phi = rospy.get_param('~weight_phi', None)

        #parameters for PID controller
        self.k_p = rospy.get_param("~k_p",None)
        self.k_i = rospy.get_param("~k_i",None)
        self.k_d = rospy.get_param("~k_d",None)
        #saturation parameters
        self.sati = rospy.get_param("~sati",None)
        self.satd = rospy.get_param("~satd",None)
        self.omegasat = rospy.get_param("~omegasat",None)

        #self.vref = rospy.get_param("~vref",None)   #v_ref defines speed at which the robot moves 
        self.vref = float(os.environ['SPEED'])


    #function to reset Integralstate, if robot is thought to be perfectly in Lane (d=phi=0)
    def resetintegral(self,d,phi):
        tol_d = 0.05
        tol_phi = 0.1

        if np.abs(d) <= tol_d and np.abs(phi) <= tol_phi:
            self.C_i = 0
            rospy.loginfo("Reset Integral")

    #returns true, if robot moves towards lanecenter and False otherwise
    def drivetocenter(self,d,phi):
        if d<0 and phi>0:
            return True
        elif d>0 and phi<0:
            return True
        else:
            return False

    #compute velocityboost based on distance to lanecenter
    def computespeed(self,d):
        speedfactor = 2.0
        return speedfactor*np.abs(d)

    #compute controlaction based on lanepose estimate
    def getcontrolaction(self,dist,phi,dt):
        
        #compute error for PID
        err = self.weight_d*dist+self.weight_phi*phi

        #proportional gain part
        C_p = self.k_p*err

        #integral term (approximate integral)
        self.C_i += self.k_i*dt*err

        #activate if integralreset is desired:
        #sets integralterm C_i to zero if d and theta are zero, thus the DB is driving perfectly in lane
        #self.resetintegral(dist,tist)

        #integral saturation
        #make sure integral term doesnt become too big
        if self.C_i > self.sati:
            self.C_i = self.sati 
        if self.C_i < -self.sati:
            self.C_i = -self.sati 
        
        #derivative term (usually not used, because noise makes it rather unstable)
        C_d = self.k_d*(err-self.dold)/dt
        self.dold = err
        
        #derivative saturation
        #make sure derivative term doesnt become too big
        if C_d > self.satd:
            C_d = self.satd
        if C_d < -self.satd:
            C_d = -self.satd
        
        #computing control output
        omega = C_p + self.C_i + C_d
        
        #output saturation 
        #making sure, that too large of actuator output is requested, 
        #since a) the real life motors cannot achieve any arbitrary speed 
        #and b) too big of omega could make the robot crash, since there is a delay in the measurements
        if omega>self.omegasat:
            omega=self.omegasat
        if omega<-self.omegasat:
            omega=-self.omegasat

        #uncomment the following part to allow change in velocity based on distance to lane center  and
        #heading direction (heading towards lane center -> drive faster, heading away -> drive slower)
        #if robot is almost perfectly in lane -> drive faster 
        '''
        if np.abs(omega) < 0.05 and np.abs(self.omega_old) < 0.05:
            v = 0.3
        elif self.drivetocenter(dist,tist):
            v = vref + self.computespeed(dist)
        else:
            v = vref - self.computespeed(dist)
        
        self.omega_old = omega
        '''

        v = self.vref
        return v,omega

    def run(self):
        # publish message every 1/x second
        rate = rospy.Rate(10) 
        car_cmd_msg = WheelsCmdStamped()
        #car_cmd_msg = Twist2DStamped()
        tnew = time.time()
        stoptime = 28.0
        t0 = time.time()
        i=0

        parammsg1 = [self.k_p,self.k_i,self.k_d]
        rospy.loginfo("[kp,ki,kd] = %s" % parammsg1)
        
        while  not rospy.is_shutdown():
            #computing dt for I-part of controller
            told = tnew
            tnew = time.time()
            dt = tnew-told
            
            '''
            #stop programm once a certain time has passed (for experiments, not meant for normal usage)
            if tnew-t0>stoptime:
                rospy.logwarn("Time's up!!!")
                rospy.signal_shutdown("Ende gut, alles gut")
                self.custom_shutdown()
            '''
            
            v,omega = self.getcontrolaction(self.dist,self.phi,dt)

            #car_cmd_msg.omega = self.omega
            #car_cmd_msg.v = self.vref

            #console output, if requested actuatoroutput saturates
            if np.abs(omega) >= self.omegasat:
                rospy.logwarn("Max Omega reached")

            #def. motor commands that will be published
            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = v - 0.5*self.baseline * omega
            car_cmd_msg.vel_right = v + 0.5*self.baseline * omega
            #publish actuator output to wheels_driver_node
            self.pub_car_cmd.publish(car_cmd_msg)

            #printing messages to verify that program is working correctly 
            #i.ei if dist and tist are always zero, then there is probably no data from the lan_pose
            message1 = self.dist
            message2 = omega
            message3 = self.phi
            message4 = v

            #rospy.loginfo('d: %s' % message1)
            #rospy.loginfo('omega: %s' % message2)
            #rospy.loginfo('phi: %s' % message3)
            rospy.loginfo('v: %s' % message4)
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

        delay = rospy.Time.now() - pose.header.stamp
        delay_float = delay.secs + float(delay.nsecs)/1e9    
        rospy.loginfo('delay [s] =  %s' % delay_float)       

if __name__ == "__main__":
    # Initialize the node
    #rospy.loginfo("Hello from the start")

    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()