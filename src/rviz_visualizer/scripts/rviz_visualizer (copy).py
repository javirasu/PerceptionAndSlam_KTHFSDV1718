#!/usr/bin/env python
import numpy as np
from visualization_msgs.msg import Marker
import tf
import rospy
from geometry_msgs.msg import PointStamped, Point
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class RVIZ_visualizer():

    def __init__(self):
        self.id_glob = 0
        self.main()

    
    def callback_reactive(self, msg):
        reactive = msg.data.reshape(int(msg.data.shape[0]/4),4)
        self.publish_reactive(reactive)
        self.publish_FOV()

    # Creates Rviz markers and publishes it
    def publish_reactive(self, reactive):
        marker_publisher = rospy.Publisher('/reactive_map_markers', Marker, queue_size=10)

        i = 0
        for cone in reactive:
            marker = Marker()
            marker.id = i
            marker.ns = "reactive"
            marker.header.frame_id = "/zed_center"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.01
            marker.color.a = 0.7

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # if cone[3] == 0:     #YELLOW
            #     marker.color.r = 1.0
            #     marker.color.g = 1.0
            #     marker.color.b = 0.0
            # elif cone[3] == 1:   #BLUE
            #     marker.color.r = 0.0
            #     marker.color.g = 0.0
            #     marker.color.b = 1.0
            # elif cone[3] == 2:   #ORANGE
            #     marker.color.r = 1.0
            #     marker.color.g = 0.5
            #     marker.color.b = 0.0       

            marker.lifetime = rospy.Duration(0.25)
            # marker.pose.position.x = cone[2]
            # marker.pose.position.y = -cone[0]
            # marker.pose.position.z = -cone[1]
            marker.pose.position.x = cone[0]
            marker.pose.position.y = cone[1]
            marker.pose.position.z = 0.01

            marker_publisher.publish(marker)
            rospy.Rate(100).sleep()
            i = i + 1

    def publish_glob(self, glob):
        marker_publisher = rospy.Publisher('/glob_map_rviz', Marker, queue_size=10)

        for i in range(self.id_glob):
            marker = Marker()
            marker.id = i
            marker.ns = "glob"
            marker.action = marker.DELETE
            marker_publisher.publish(marker)
            rospy.Rate(100).sleep()

            
        self.id_glob = 0
        for cone in glob:
            marker = Marker()
            marker.id = self.id_glob
            marker.ns = "glob"
            marker.header.frame_id = "/map"
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = cone[2]
            marker.scale.y = cone[2]
            marker.scale.z = 0.01

            if cone[3] == 0:     #YELLOW
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif cone[3] == 1:   #BLUE
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif cone[3] == 2:   #ORANGE
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0         


            if cone[5] == 1:
                marker.color.a = 1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0  
            else:
                marker.color.a = 0.2
               
            # marker.lifetime = rospy.Duration(0.25)
            # marker.pose.position.x = cone[2]
            # marker.pose.position.y = -cone[0]
            # marker.pose.position.z = -cone[1]
            marker.pose.position.x = cone[0]
            marker.pose.position.y = cone[1]
            marker.pose.position.z = 0.01

            marker_publisher.publish(marker)
            rospy.Rate(100).sleep()
            self.id_glob = self.id_glob + 1

    def publish_FOV(self):
        self.dist_FOV = 4.0
        self.angle = 45

        marker_publisher = rospy.Publisher('/FOV_marker', Marker, queue_size=10)
        marker = Marker()
        marker.id = 1500
        marker.ns = "FOV"
        marker.header.frame_id = "/zed_left_camera"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        pA = Point() ; pA.x = 0 ; pA.y = 0 ; pA.z = 0
        pB = Point() ; pB.x = self.dist_FOV * np.cos(self.angle * np.pi / 180) ; pB.y = self.dist_FOV * np.sin(self.angle * np.pi / 180) ; pB.z = 0
        pC = Point() ; pC.x = self.dist_FOV * np.cos(-self.angle * np.pi / 180) ; pC.y = self.dist_FOV * np.sin(-self.angle * np.pi / 180) ; pC.z = 0

        p_list = [pA, pB, pB, pC, pC, pA]

        marker.points = p_list
        

        marker_publisher.publish(marker)
        rospy.Rate(100).sleep()

    def callback_global(self, msg):
        try:
            glob = msg.data.reshape(int(msg.data.shape[0]/6),6)
            self.publish_glob(glob)
        except:
            return

    

    def main(self):
        rospy.init_node('rviz_visualizer_node')
        rospy.Subscriber('/reactive_cones', numpy_msg(Floats), self.callback_reactive)
        rospy.Subscriber('/global_map_markers', numpy_msg(Floats), self.callback_global)
        rospy.spin()

RVIZ_visualizer()