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
        self.alpha = 0.9
        self.max_hits = 100
        self.min_hits = -5
        self.R_max_cov = 0.5
        self.R_min_cov = 0.1
        self.freq_output = 10
        self.delta_cov = self.R_max_cov * (self.max_hits - 1) / self.max_hits**2
        # -----

        self.cone_db = np.array([])
        self.listener = tf.TransformListener()
        self.dist_FOV_max = 4.0
        self.dist_FOV_min = 1.7
        self.last_odom = Odometry()
        self.bias_left_lense = -0.06
        self.freq_update_db = 10
        self.add_hit = 3
        self.sub_hit = 2
        self.angle_FOV = 45
        
        self.transformed_cones = []

        self.legit_cone_hits  = 20
        self.freq_clean_db = 1

        # -----

        self.id_cone = 0

        self.main()

    def callback_odom(self, msg):
        self.last_odom = msg
        self.T = np.array((msg.pose.pose.position.x , msg.pose.pose.position.y)).reshape(1,2)
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.theta = y

        self.R_plus = np.array([[np.cos(self.theta), -np.sin(self.theta)],[np.sin(self.theta), np.cos(self.theta)]]).reshape(2,2)
        self.R_minus = np.array([[np.cos(-self.theta), -np.sin(-self.theta)],[np.sin(-self.theta), np.cos(-self.theta)]]).reshape(2,2)

        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, y], dtype=np.float32)

    def callback_reactive(self, msg):
        last_reactive = msg.data.reshape(int(msg.data.shape[0]/4),4)
        last_reactive = np.delete(last_reactive, 2, axis=1)
        self.reactive_cones = copy.deepcopy(last_reactive)

    def publish_2_icp(self, react, db, pose):
        react = copy.deepcopy(react)
        db = copy.deepcopy(db)
        pose = copy.deepcopy(pose)

        if db.shape[0] != 1:
            db = db.squeeze()
        elif  db.shape[0] == 1:
            db = db.squeeze()
            db = db.reshape(1,2)

        if db.shape[0] != react.shape[0]:
            return
        
        data = np.zeros((1+db.shape[0]+react.shape[0]))


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

            if dist <= self.dist_FOV_max and angle <= self.angle_FOV and angle >= -self.angle_FOV:
                reactive_in_fov_index.append(i)

        # [x,y,color]
        reactive_in_fov_local = copy.deepcopy(reactive_cones[reactive_in_fov_index,:])

        # [x,y,color,cov,hits,inFOV, id]
        reactive_in_fov_local = np.hstack((reactive_in_fov_local, np.zeros((reactive_in_fov_local.shape[0], 4))))
        # Assign max covariance to cones
        reactive_in_fov_local[:,3] = self.R_max_cov

        reactive_in_fov_map = copy.deepcopy(reactive_in_fov_local)

        if reactive_in_fov_map.shape[0] > 1:
            reactive_in_fov_map[:,0:2] = np.matmul(reactive_in_fov_map[:,0:2] , R_minus)
            reactive_in_fov_map[:,0:2] = reactive_in_fov_map[:,0:2] + T
        elif reactive_in_fov_map.shape[0] == 1:
            reactive_in_fov_map.reshape(1,7)
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
        
        # If cones in db, add hits and ignore those too close to a seen cone
        reactive_in_db = []
        db_in_reactive = []
        for i in range(reactive_in_fov_map.shape[0]):
            reactive = reactive_in_fov_map[i]
            dist_db = np.linalg.norm(self.cone_db[:,0:2]-reactive[0:2] ,axis=1)
            if np.amin(dist_db) <= self.R_max_cov:
                reactive_in_db.append(i)
                db_in_reactive.append(np.argmin(dist_db))
        reactive_in_fov_map_new = np.delete(reactive_in_fov_map, reactive_in_db, axis=0)

        # Data for ICP
        dataICP = copy.deepcopy(self.pose).reshape(1,3)
        for i in range(len(reactive_in_db)):
            i_react = reactive_in_db[i]
            i_db = db_in_reactive[i]
            react = np.array([reactive_in_fov_map[i_react,0], reactive_in_fov_map[i_react,1],0]).reshape(1,3)
            db = np.array([self.cone_db[i_db,0],self.cone_db[i_db,1],0]).reshape(1,3)

            dataICP = np.vstack((dataICP, db, react))
        
        # Send to ICP
        if len(reactive_in_db) > 3:
            icp_publisher = rospy.Publisher('/icp_input', numpy_msg(Floats), queue_size=10)
            icp_publisher.publish(dataICP.flatten())
            



        # Add hits to db cones seen in the reactive map
        self.cone_db[db_in_reactive,4] = self.cone_db[db_in_reactive,4] + self.add_hit
        self.cone_db[np.argwhere(self.cone_db[:,4] > self.max_hits),4] = self.max_hits
        self.cone_db[np.argwhere(self.cone_db[:,4] < self.min_hits),4] = self.min_hits
        # Update position with moving average
        self.cone_db[db_in_reactive,0:2] = (1-self.alpha) * reactive_in_fov_map[reactive_in_db,0:2] + (self.alpha) * self.cone_db[db_in_reactive,0:2]
        # Update covariance
        self.cone_db[:,3] = self.R_max_cov - self.delta_cov * self.cone_db[:,4]
        self.cone_db[np.argwhere(self.cone_db[:,3] > self.R_max_cov),3] = self.R_max_cov
        self.cone_db[np.argwhere(self.cone_db[:,3] < self.R_min_cov),3] = self.R_min_cov

        # Add the new cones to the db
        self.cone_db = np.vstack((self.cone_db, reactive_in_fov_map_new))

        # Give id to cones with id == 0
        no_id_index = np.argwhere(self.cone_db[:,6] == 0)
        for id_cone in no_id_index:
            self.cone_db[id_cone,6] = self.id_cone
            self.id_cone = self.id_cone + 1


        # Calculate cones in db and FOV
        db_local = copy.deepcopy(self.cone_db)

        if db_local.shape[0] > 1:
            db_local[:,0:2] = db_local[:,0:2] - T
            db_local[:,0:2] = np.matmul(db_local[:,0:2] , R_plus)
        elif db_local.shape[0] == 1:
            db_local.reshape(1,7)
            db_local[0, 0:2] = db_local[0, 0:2] - T
            db_local[0, 0:2] = np.matmul(db_local[0, 0:2] , R_plus)

        db_local_distance = np.linalg.norm(db_local[:,0:2], axis=1)
        db_local_in_distance_index = np.argwhere(np.logical_and(db_local_distance <= self.dist_FOV_max, db_local_distance >= self.dist_FOV_min))

        db_local_in_angle_index = []

        for index_cone in db_local_in_distance_index:
            x = db_local[index_cone,0]
            y = db_local[index_cone,1]
            angle = np.arctan2(y,x) * 180 / np.pi

            if angle <= self.angle_FOV and angle >= -self.angle_FOV:
               db_local_in_angle_index.append(index_cone) 

        self.cone_db[db_local_in_angle_index,5] = 1

        for index in db_local_in_angle_index:
            if index not in db_in_reactive[:]:
                self.cone_db[index,4] = self.cone_db[index,4] - self.sub_hit

        self.cone_db = np.delete(self.cone_db, np.argwhere(self.cone_db[:,4] <= self.min_hits), axis=0)
        
        self.output_4_rviz(self.cone_db.astype(np.float32))
        
        
        self.cone_db[:,5] = 0
        
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
