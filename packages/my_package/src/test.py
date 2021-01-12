#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelsCmdStamped, LanePose, Twist2DStamped
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image


class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        #self.pub = rospy.Publisher('/fakebot/lane_filter_node/lane_pose', LanePose, queue_size=10)
        self.sub = rospy.Subscriber('/fakebot/lane_controller_node/car_cmd', Twist2DStamped, self.callback)

        self.pub_actuator_cmd = rospy.Publisher("fakebot/sim_node/actuator_cmd",WheelsCmdStamped, queue_size=1)

        self.sub_debug_lanepose = rospy.Subscriber("fakebot/sim_node/debug_lanepose",LanePose,self.cb_debug_pose,queue_size=1)

        self.msg = WheelsCmdStamped()

        self.L = 0.05

    def cb_debug_pose(self,msg):
        pose = [msg.d, msg.phi]
        rospy.loginfo("pose = %s" % pose)
        

    def callback(self,msg):
        #rospy.loginfo("Hallo 4")
        act_msg = WheelsCmdStamped()
        act_msg.header.stamp = msg.header.stamp
        act_msg.vel_left = msg.v - self.L* msg.omega
        act_msg.vel_right = msg.v + self.L * msg.omega

        #dt = rospy.Time.now() - msg.header.stamp
        #rospy.loginfo("dt = %s" % dt)

        #rospy.loginfo("v_ref = %s" % msg.v)
        #rospy.loginfo("omega = %s" % msg.omega)

        #act_msg.vel_left = -0.1
        #act_msg.vel_right = 0.1

        self.pub_actuator_cmd.publish(act_msg)



    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        

        while not rospy.is_shutdown():
            

            #rospy.loginfo("Hallo 3")
            #self.pub.publish(lanepose)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    #node.run()
    # keep spinning
    rospy.spin()
