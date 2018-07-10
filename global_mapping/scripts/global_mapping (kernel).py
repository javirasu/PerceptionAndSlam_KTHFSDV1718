#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import PointStamped
import tf
import time
import copy

class Global_Mapping_Node():

    def __init__(self):
        # Global variables
        self.alpha = 0.8
        self.max_hits = 50
        self.R_max_cov = 0.8
        self.R_min_cov = 0.1
        self.freq_output = 10
        self.delta_cov = self.R_max_cov * (self.max_hits - 1) / self.max_hits**2
        # -----

        self.cone_db = np.array([])
        self.listener = tf.TransformListener()
        self.dist_FOV = 4.0
        self.last_odom = Odometry()
        self.bias_left_lense = -0.06
        self.freq_update_db = 10
        self.add_hit = 7
        self.sub_hit = 2
        self.angle_FOV = 45
        
        self.transformed_cones = []

        self.legit_cone_hits  = 20
        self.freq_clean_db = 1

        # -----

        self.main()

    def callback_odom(self, msg):
        self.last_odom = msg
        self.T = np.array((msg.pose.pose.position.x , msg.pose.pose.position.y)).reshape(1,2)
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        theta = y

        self.R_plus = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]]).reshape(2,2)
        self.R_minus = np.array([[np.cos(-theta), -np.sin(-theta)],[np.sin(-theta), np.cos(-theta)]]).reshape(2,2)

    def callback_reactive(self, msg):
        last_reactive = msg.data.reshape(int(msg.data.shape[0]/4),4)
        last_reactive = np.delete(last_reactive, 2, axis=1)
        self.reactive_cones = copy.deepcopy(last_reactive)
        # R_minus = self.R_minus
        # T = self.T

        # try:

        #     if self.transformed_cones.shape[0] > 1:
        #         self.transformed_cones[:,0:2] = np.matmul(self.transformed_cones[:,0:2] , R_minus)
        #         self.transformed_cones[:,0:2] = self.transformed_cones[:,0:2] + T
        #     elif self.transformed_cones.shape[0] == 1:
        #         self.transformed_cones.reshape(1,4)
        #         self.transformed_cones[0, 0:2] = np.matmul(self.transformed_cones[0, 0:2] , R_minus)
        #         self.transformed_cones[0, 0:2] = self.transformed_cones[0, 0:2] + T
    
        # except:
        #     return


    def update_cone_db(self, event):

        # Get reactive cones only in FOV
        try:
            reactive_cones = self.reactive_cones
            R_plus = self.R_plus
            R_minus = self.R_minus
            T = self.T
        except:
            print("No ready yet")
            return

        reactive_in_fov_index = []

        # Check cones on reactive which are also in the FOV
        for i in range(reactive_cones.shape[0]):
            c = reactive_cones[i,:]

            angle = (np.arctan2(c[1],c[0]) * 180 / np.pi) 
            dist = np.sqrt(c[1]**2 + c[0]**2)

            if dist <= self.dist_FOV and angle <= self.angle_FOV and angle >= -self.angle_FOV:
                reactive_in_fov_index.append(i)

        # [x,y,color]
        reactive_in_fov_local = copy.deepcopy(reactive_cones[reactive_in_fov_index,:])

        # [x,y,color,cov,hits,inFOV]
        reactive_in_fov_local = np.hstack((reactive_in_fov_local, np.zeros((reactive_in_fov_local.shape[0], 3))))

        reactive_in_fov_map = copy.deepcopy(reactive_in_fov_local)

        if reactive_in_fov_map.shape[0] > 1:
            reactive_in_fov_map[:,0:2] = np.matmul(reactive_in_fov_map[:,0:2] , R_minus)
            reactive_in_fov_map[:,0:2] = reactive_in_fov_map[:,0:2] + T
        elif reactive_in_fov_map.shape[0] == 1:
            reactive_in_fov_map.reshape(1,6)
            reactive_in_fov_map[0, 0:2] = np.matmul(reactive_in_fov_map[0, 0:2] , R_minus)
            reactive_in_fov_map[0, 0:2] = reactive_in_fov_map[0, 0:2] + T


        # If db empty, add all reactive cones
        if self.cone_db.shape[0] == 0:
            print("Empty db, adding new cones")
            self.cone_db = copy.deepcopy(reactive_in_fov_map)
            self.output_4_rviz(self.cone_db.astype(np.float32))
            return
        else:
            print("Cones in db: "+str(self.cone_db.shape[0]))
        
        # If cones in db, remove those too close to a seen cone
        reactive_to_delete = []
        for i in range(reactive_in_fov_map.shape[0]):
            reactive = reactive_in_fov_map[i]
            dist_db = np.linalg.norm(self.cone_db[:,0:2]-reactive[0:2] ,axis=1)
            if np.amin(dist_db) <= self.R_max_cov:
                reactive_to_delete.append(i)
        reactive_in_fov_map_new = np.delete(reactive_in_fov_map, reactive_to_delete, axis=0)

        # Add the new cones to the db
        self.cone_db = np.vstack((self.cone_db, reactive_in_fov_map_new))
        
        self.output_4_rviz(self.cone_db.astype(np.float32))

        print("---- Cone db ----")
        i = 0
        for cone in self.cone_db:
            print(str(i)+ ":  "+ str(cone[0])+ " "+ str(cone[1]) + " "+ str(cone[2]) + " "+ str(cone[3]) + " "+ str(cone[4]) + " "+ str(cone[5]))
            i = i + 1
        
        
        
    def output_4_rviz(self, rviz_cones): 
        pub_rviz = rospy.Publisher('/global_map_markers', numpy_msg(Floats), queue_size=10)
        pub_rviz.publish(rviz_cones.flatten())
            


    def main(self):
        rospy.init_node('mapping_node')
        rospy.Subscriber('/reactive_cones', numpy_msg(Floats), self.callback_reactive)
        rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)
        # rospy.Timer(rospy.Duration(1.0/self.freq_output), self.output_4_nav)
        rospy.Timer(rospy.Duration(1.0/self.freq_update_db), self.update_cone_db)
        # rospy.Timer(rospy.Duration(1.0/self.freq_clean_db), self.clean_cone_db)
        rospy.spin()

Global_Mapping_Node()
