#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, LanePose, SegmentList
import numpy as np
import time
import math

class ControllerNode(DTROS):
    def __init__(self,node_name):

        super(ControllerNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        self.vref = 0.0
        self.baseline = 0.1
        self.omega = 0.0
        self.tnew = 0.0

        self.dist = 0.0
        self.phi = 0.0

        # Subscriptions
        #subscribe to the node publishing the detected color segments
        self.sub_seg = rospy.Subscriber("fakebot/ground_projection_node/lineseglist_out", SegmentList, self.process_segments, queue_size=1)
        
        #self.sub_pose = rospy.Subscriber(str(os.environ['VEHICLE_NAME'])+"/lane_filter_node/lane_pose", LanePose ,self.updatepose ,queue_size = 1)
        # Publication
        #Publishes actuator commands to node handling the wheel commands
        self.pub_wheels_cmd = rospy.Publisher("fakebot/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1,dt_topic_type=TopicType.CONTROL)

        # Stop on shutdown
        rospy.on_shutdown(self.custom_shutdown)



    def custom_shutdown(self):
        stop_msg = WheelsCmdStamped()
        stop_msg.header.stamp = rospy.get_rostime()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0

        self.pub_wheels_cmd.publish(stop_msg)
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown complete oder?")


    #every time a segmentlist is received, this function is executed
    def process_segments(self, input_segment_list):
        all_segments = input_segment_list.segments # this is a list of type Segment

        #function params
        lookahead = 0.25    #parameter at which mean distance the controller looks for segments
        tol = 0.1           #the range in which the controller looks for segments

        num_yellow = 0      #amount of yellow segments in range
        num_white = 0       #amount of white segments in range     

        num_yellow_total = 0    #total amount of yellow segments 
        num_white_total = 0     #tital amount of white segments

        yellow_arr_total = np.zeros(2)  #average location of all yellow segments
        white_arr_total = np.zeros(2)   #average location of all white segments

        yellow_arr = np.zeros(2)        #average location of yellow segments in range
        white_arr = np.zeros(2)         #average location of white segments in range

        flag = True  #to keep track if there are any segments detected (True - yes ; False - no)

        #loop through all segment
        for segment in all_segments:
            #location of the two corners definig the segment
            point0 = segment.points[0]
            point1 = segment.points[1]

            #taking the average of the corner points to get center point of segment
            ave_point_x = (point0.x + point1.x)/2.0
            ave_point_y = (point0.y + point1.y)/2.0

            #compute distance to segment
            d = np.sqrt(ave_point_x**2 + ave_point_y**2)

            #if the detected segment is yellow
            if segment.color == 1:    
                
                #add segment information to params
                num_yellow_total += 1      
                yellow_arr_total += np.array([ave_point_x,ave_point_y])

                #if segment in range, keep track of it for control output
                if d < lookahead + tol and d > lookahead - tol:
                    num_yellow += 1
                    yellow_arr += np.array([ave_point_x,ave_point_y])

            #if the detected segment is white
            if segment.color == 0 and ave_point_y < 0.1 and ave_point_y > -0.15:    
                
                #add segment information to params
                num_white_total += 1
                white_arr_total += np.array([ave_point_x,ave_point_y])

                #if segment in range, keep track of it for control output
                if d < lookahead + tol and d > lookahead - tol: #and ave_point_y > -0.1:
                    num_white += 1
                    white_arr += np.array([ave_point_x,ave_point_y])

        #if both yellow and white segments are detected
        if num_white != 0 and num_yellow != 0:

            #compute average of all detected segments (per color)
            ave_white = white_arr * 1. / num_white
            ave_yellow = yellow_arr * 1. / num_yellow

            #taking the color average, compute the centerpoint of the two averages
            ave_point = (ave_white + ave_yellow)/2.0

            #set higher velocity if both segments are detected, (which means, that the robot is likely to be near the center of the lane)
            self.vref = 0.23

            '''
            if np.abs(ave_point[1]) < 0.025:
                self.vref = 0.4
                rospy.logwarn("speed")
            '''

        #if no yellow/white segments are detected in range, should only happen if:
        #either the robot crashed (outside of lane)
        #or lighting conditions are too bad, thus the segments (while in frame) are not detected (correctly)
        #or the parameters are not chosen correct
        if num_white == 0 and num_yellow == 0:
            #no white/yellow segments detected 
            #turn on the spot, until segments are detected
            self.vref = 0.0
            self.omega = 2.5

            '''
            if white_arr_total[1] < 0 and yellow_arr_total[1] > 0:
                self.omega = 0.0
                self.vref = 0.1
            elif white_arr_total[1] > 0 and yellow_arr_total[1] < 0:
                self.omega = -2.0
                self.vref = 0.1
            elif white_arr_total[1] > 0 and yellow_arr_total[1] == 0:
                self.omega = 2.0
                self.vref = 0.1
            elif num_yellow_total !=0 and num_white_total == 0:
                self.omega = -1.5
                self.vref = 0.1
            elif num_white_total !=0 and num_yellow_total == 0:
                self.omega = 1.5
                self.vref = 0.1
            else:
                self.omega = 1.0
                self.vref = 0.0
            '''


            flag = False

        #if only yellow segment are detected, meaning, that the robot is likely to be too far to the left
        # -> turn right: take average of yellow segments + some offset towards to the center of the lane
        if num_white == 0 and num_yellow != 0:

            ave_yellow = yellow_arr * 1. / num_yellow

            #offset to the right, to get robot back in lane
            offset = -0.15
            ave_point = ave_yellow + np.array([0.0,offset])

            #smaller velocity, since robot is not optimaly in lane
            self.vref = 0.20

        #if only white segment are detected, meaning, that the robot is likely to be too far to the right
        # -> turn left: take average of white segments + some offset towards to the center of the lane        
        if num_yellow == 0 and num_white != 0:
            #only white segments (probably too far to the right of the lane)

            ave_white = white_arr * 1. / num_white

            #offset to the left, to get robot back in lane
            offset = 0.35

            #if robot is in a tight turn, this statement usually true, giving a bigger offset to get a higher 
            #rotational velocity, to get the turn more smoothely
            if self.omega > 3.0:
                offset += 0.1
            
            ave_point = ave_white + np.array([0.0,offset])

            #smaller velocity, since robot is not optimaly in lane
            self.vref = 0.20

        #executed if yellow/white segments are detected in range
        if flag == True:
            #taking relative position of segments with respect to robot, compute angle between heading direction 
            #and direct path to the computed targetpoint
            alpha = np.arctan2(ave_point[1],ave_point[0])

            #d = np.sqrt(ave_point[0]**2 + ave_point[1]**2)

            #compute actuator output
            self.omega = 4.8*self.vref* np.sin(alpha)/lookahead

            rospy.loginfo("target: %s" % ave_point)
            #rospy.loginfo("dist to target: %s" % d)
           

    
    def run(self):
        rate = rospy.Rate(20) 
        while not rospy.is_shutdown():
            car_cmd_msg = WheelsCmdStamped()

            car_cmd_msg.header.stamp = rospy.get_rostime()
            car_cmd_msg.vel_left = self.vref - 0.5*self.baseline * self.omega
            car_cmd_msg.vel_right = self.vref + 0.5*self.baseline * self.omega

            # Send the command to the car
            self.pub_wheels_cmd.publish(car_cmd_msg)
            rospy.loginfo("omega = %s" % self.omega)
            
            rate.sleep()

if __name__ == '__main__':
    # Initialize the node
    lane_controller_node = ControllerNode(node_name='lane_controller_node')

    lane_controller_node.run()
    # Keep it spinning
    rospy.spin()
