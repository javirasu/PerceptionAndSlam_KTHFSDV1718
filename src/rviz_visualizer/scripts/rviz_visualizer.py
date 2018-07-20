#!/usr/bin/env python
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import tf
import rospy
from geometry_msgs.msg import PointStamped, Point
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
import rostopic

class RVIZ_visualizer():

    def __init__(self):
        self.lifetime = 1.0
        self.id_glob = 0
        self.main()

    
    def callback_reactive(self, msg):
        reactive = msg.data.reshape(int(msg.data.shape[0]/4),4)
        self.publish_reactive(reactive)

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

    def callback_odom(self, msg):

        self.T = np.array((msg.pose.pose.position.x , msg.pose.pose.position.y)).reshape(1,2)
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        theta = y
        self.R_plus = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]]).reshape(2,2)
        self.R_minus = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta), np.cos(-theta)]]).reshape(2,2)

    def publish_glob(self, glob):
        marker_publisher = rospy.Publisher('/glob_map_rviz', MarkerArray, queue_size=10)

        msg = MarkerArray()
        markers = []

        i = 0
        for cone in glob:
            marker = Marker()
            # marker.id = np.random.randint(0,1e6)
            marker.id = int(cone[6])
            marker.ns = "glob"
            marker.header.frame_id = "/map"
            marker.type = marker.CYLINDER
            if int(cone[4]) > -4:
                marker.action = marker.ADD
            else:
                marker.action = marker.DELETE
            marker.scale.x = cone[3]
            marker.scale.y = cone[3]
            marker.scale.z = 0.01

            if cone[5] == 1:
                marker.color.a = 1 
            else:
                marker.color.a = 0.4

            if int(cone[2]) == 0.0:     #YELLOW
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif int(cone[2]) == 1.0:   #BLUE
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif int(cone[2]) == 2.0:   #ORANGE
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0   

            marker.pose.position.x = cone[0]
            marker.pose.position.y = cone[1]
            marker.pose.position.z = 0.01

            marker.lifetime = rospy.Duration(0.5)

            markers.append(marker)

        msg.markers = markers
        marker_publisher.publish(msg)
        rospy.Rate(100).sleep()
        i = i + 1

    def publish_FOV(self, event):
        try:
            self.dist_FOV = 4.0
            self.angle = 45
            self.delta_point_FOV = 22.5

            A_local = np.array([0,0])
            B_local = np.array([self.dist_FOV * np.cos(self.angle * np.pi / 180) , self.dist_FOV * np.sin(self.angle * np.pi / 180) ])
            C_local = np.array([self.dist_FOV * np.cos((self.angle-self.delta_point_FOV) * np.pi / 180) , self.dist_FOV * np.sin((self.angle-self.delta_point_FOV) * np.pi / 180) ])
            D_local = np.array([self.dist_FOV * np.cos((self.angle-2*self.delta_point_FOV) * np.pi / 180) , self.dist_FOV * np.sin((self.angle-2*self.delta_point_FOV) * np.pi / 180) ])
            E_local = np.array([self.dist_FOV * np.cos((self.angle-3*self.delta_point_FOV) * np.pi / 180) , self.dist_FOV * np.sin((self.angle-3*self.delta_point_FOV) * np.pi / 180) ])
            F_local = np.array([self.dist_FOV * np.cos((self.angle-4*self.delta_point_FOV) * np.pi / 180) , self.dist_FOV * np.sin((self.angle-4*self.delta_point_FOV) * np.pi / 180) ])
            

            A_map = A_local
            B_map = B_local
            C_map = C_local
            D_map = D_local
            E_map = E_local
            F_map = F_local

            # A_map = A_local + self.T
            A_map = np.matmul(A_map , self.R_minus)
            A_map = A_map + self.T
            # B_map = B_local + self.T
            B_map = np.matmul(B_map , self.R_minus)
            B_map = B_map + self.T
            # C_map = C_local + self.T
            C_map = np.matmul(C_map , self.R_minus)
            C_map = C_map + self.T

            D_map = np.matmul(D_map , self.R_minus)
            D_map = D_map + self.T

            E_map = np.matmul(E_map , self.R_minus)
            E_map = E_map + self.T

            F_map = np.matmul(F_map , self.R_minus)
            F_map = F_map + self.T

            A_map = A_map.flatten()
            B_map = B_map.flatten()
            C_map = C_map.flatten()
            D_map = D_map.flatten()
            E_map = E_map.flatten()
            F_map = F_map.flatten()

            marker_publisher = rospy.Publisher('/FOV_marker', Marker, queue_size=10)
            marker = Marker()
            marker.id = 1500
            marker.ns = "FOV"
            marker.header.frame_id = "/map"
            marker.type = marker.LINE_LIST
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            pA = Point() ; pA.x = A_map[0] ; pA.y = A_map[1] ; pA.z = 0
            pB = Point() ; pB.x = B_map[0]  ; pB.y = B_map[1] ; pB.z = 0
            pC = Point() ; pC.x = C_map[0] ; pC.y = C_map[1] ; pC.z = 0
            pD = Point() ; pD.x = D_map[0] ; pD.y = D_map[1] ; pD.z = 0
            pE = Point() ; pE.x = E_map[0] ; pE.y = E_map[1] ; pE.z = 0
            pF = Point() ; pF.x = F_map[0] ; pF.y = F_map[1] ; pF.z = 0

            p_list = [pA, pB, pB, pC, pC, pD, pD, pE, pE, pF, pF, pA]

            marker.points = p_list
            

            marker_publisher.publish(marker)
            rospy.Rate(100).sleep()
        except:
            return


    def callback_global(self, msg):
        data = msg.data.astype(np.float32)
        try:
            glob = msg.data.reshape(int(data.shape[0]/7),7)
            self.publish_glob(glob)
        except:
            return

    def lifetime_global(self, msg):
        # self.r.print_hz(['/reactive_cones'])
        topic_info = self.r.get_hz(topic='/global_map_markers')

        try:
            self.lifetime = 1/topic_info[0]
        except:
            return

    def main(self):
        rospy.init_node('rviz_visualizer_node')
        self.r = rostopic.ROSTopicHz(-1)
        rospy.Subscriber('/reactive_cones', numpy_msg(Floats), self.r.callback_hz, callback_args='/global_map_markers')
        rospy.Subscriber('/reactive_cones', numpy_msg(Floats), self.callback_reactive)
        rospy.Subscriber('/global_map_markers', numpy_msg(Floats), self.callback_global)
        rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)
        rospy.Timer(rospy.Duration(0.1), self.publish_FOV)
        rospy.Timer(rospy.Duration(0.1), self.lifetime_global)

        rospy.spin()

RVIZ_visualizer()