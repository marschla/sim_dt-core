#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import  LanePose
from numpy import random


class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        #self.pub = rospy.Publisher('/fakebot/lane_filter_node/lane_pose', LanePose, queue_size=10)
        self.sub = rospy.Subscriber('sim_node/lane_pose', LanePose, self.callback,"lane_filter",queue_size=1)

        self.pub_actuator_cmd = rospy.Publisher("~pose_out",LanePose, queue_size=1)

        self.scalefactor = os.environ['SCALE']


    def callback(self,msg,source):
        #introduces some error into lanepose to mimic uncertainty in state estimation
        random.seed()
        offset_d = (random.rand() - 0.5)/50.0*float(self.scalefactor)  #maxoffset = 0.1*factor
        offset_phi = (random.rand()-0.5)/2.86*float(self.scalefactor)  #maxoffset = 10°*factor

        pose_msg = LanePose()
        pose_msg.header = msg.header
        pose_msg.d = msg.d + offset_d
        pose_msg.phi = msg.phi + offset_phi

        self.pub_actuator_cmd.publish(pose_msg)


if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    #node.run()
    # keep spinning
    rospy.spin()
