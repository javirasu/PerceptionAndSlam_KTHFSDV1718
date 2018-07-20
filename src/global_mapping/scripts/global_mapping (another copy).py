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

        self.transformed_cones = copy.deepcopy(last_reactive)
        self.no_transformed_cones = copy.deepcopy(last_reactive)
        R_minus = self.R_minus
        T = self.T

        try:

            if self.transformed_cones.shape[0] > 1:
                self.transformed_cones[:,0:2] = np.matmul(self.transformed_cones[:,0:2] , R_minus)
                self.transformed_cones[:,0:2] = self.transformed_cones[:,0:2] + T
            elif self.transformed_cones.shape[0] == 1:
                self.transformed_cones.reshape(1,4)
                self.transformed_cones[0, 0:2] = np.matmul(self.transformed_cones[0, 0:2] , R_minus)
                self.transformed_cones[0, 0:2] = self.transformed_cones[0, 0:2] + T
    
        except:
            return


    def update_cone_db(self, event):

        print("Update db")

        reactive_cones = copy.deepcopy(self.transformed_cones)
        # Get reactive cones only in FOV
        try:
            R_plus = self.R_plus
            T = self.T
        except:
            print("No odom")
            return

        reactive_FOV = np.array(copy.deepcopy(reactive_cones))
        reactive_FOV = reactive_FOV.reshape(reactive_FOV.shape[0],4)

        if reactive_FOV.shape[0] > 1:
            reactive_FOV[:,0:2] = np.matmul(reactive_FOV[:,0:2] , R_plus)
            reactive_FOV[:,0:2] = reactive_FOV[:,0:2] - T
        elif reactive_FOV.shape[0] == 1:
            reactive_FOV[0,0:2] = np.matmul(reactive_FOV[0,0:2] , R_plus)
            reactive_FOV[0,0:2] = reactive_FOV[0,0:2] - T

        reactive_in_fov_index = []

        print("cones")

        # Check cones on reactive which are also in the FOV
        for i in range(reactive_FOV.shape[0]):
            c = reactive_FOV[i,:]

            angle = (np.arctan2(c[1],c[0]) * 180 / np.pi) 
            dist = np.sqrt(c[1]**2 + c[0]**2)

            if dist <= self.dist_FOV and angle <= self.angle_FOV and angle >= -self.angle_FOV:
                reactive_in_fov_index.append(i)

        cones = copy.deepcopy(reactive_FOV[reactive_in_fov_index,:])

        print(cones)
        

        # Compute db in FOV

        db_in_fov_index = []

        if self.cone_db.shape[0] > 0:

            R_plus = self.R_plus
            T = self.T

            local_db = copy.deepcopy(self.cone_db)
            local_db[:,0:2] = np.matmul(local_db[:,0:2] , R_plus)
            local_db[:,0:2] = local_db[:,0:2] - T

            # Check cones on db which are also in the FOV
            db_in_fov_index = []
            for i in range(local_db.shape[0]):
                c = local_db[i,:]

                angle = (np.arctan2(c[1],c[0]) * 180 / np.pi) 
                dist = np.sqrt(c[1]**2 + c[0]**2)

                if dist <= self.dist_FOV and angle <= self.angle_FOV and angle >= -self.angle_FOV:
                    db_in_fov_index.append(i)

            db_in_fov = copy.deepcopy(self.cone_db[db_in_fov_index,:])
            db_in_fov_index = np.array(db_in_fov_index)
            # Associated column
            ass_column = np.zeros((db_in_fov.shape[0], 1))
            db_in_fov = np.hstack((db_in_fov, ass_column))

        
            for ic in range(cones.shape[0]):
                c = cones[ic]
                # Reactive cone in the FOV

                local_reactive_cone = cones[ic]
                local_reactive_cone[0:2] = np.matmul(local_reactive_cone[0:2] , self.R_plus)
                local_reactive_cone[0:2] = local_reactive_cone[0:2] - self.T
                
                reactive_angle = (np.arctan2(local_reactive_cone[1], local_reactive_cone[0]) * 180 / np.pi) 
                reactive_dist = np.sqrt(local_reactive_cone[1]**2 + local_reactive_cone[0]**2)

                if reactive_dist > self.dist_FOV or reactive_angle > self.angle_FOV or reactive_angle < -self.angle_FOV:
                    continue

                # Data association
                dist_db = np.linalg.norm(db_in_fov[:,0:2]-c[0:2] ,axis=1)
                if dist_db.shape[0] > 0 and np.amin(dist_db) <= self.R_max_cov:
                    i = np.argmin(dist_db)
                    # Moving Avg.
                    db_in_fov[i,0:2] = self.alpha * db_in_fov[i,0:2] + (1-self.alpha) * c[0:2]
                    # Color
                    db_in_fov[i,3] = round(c[3])
                    # Hits
                    db_in_fov[i,4] = min(db_in_fov[i,4]+self.add_hit, self.max_hits)
                    # Covariance
                    db_in_fov[i,2] = max(self.R_max_cov - self.delta_cov * db_in_fov[i,4], self.R_min_cov)
                    # Associated
                    db_in_fov[i,5] = 1

                else:
                    new_cone = np.array([c[0],c[1], self.R_max_cov, c[3],self.add_hit])
                    if self.cone_db.shape[0] > 0:
                        self.cone_db = np.vstack((self.cone_db,new_cone))
                    else:
                        self.cone_db = new_cone.reshape(1,5)

            # Increase cov cone
            for i in range(db_in_fov.shape[0]):
                if  db_in_fov[i,5] == 0:
                    # Hits
                    db_in_fov[i,4] = db_in_fov[i,4]-self.sub_hit
                    # Covariance
                    db_in_fov[i,2] = min(self.R_max_cov - self.delta_cov * db_in_fov[i,4], self.R_max_cov)

            # Update db
            for i in range(db_in_fov.shape[0]):
                if db_in_fov.shape[0] > 1:
                    db_index = db_in_fov_index[i]
                else:
                    db_index = np.asscalar(db_in_fov_index)

                self.cone_db[db_index, :] = db_in_fov[i,0:5]
            
            if db_in_fov.shape[0] > 1:
                remove_index = db_in_fov_index[np.argwhere(db_in_fov[:,4] <= -5 )]
            else:
                if db_in_fov[:,4] <= -5:
                    remove_index = np.asscalar(db_in_fov_index)
                else:
                    remove_index = []

            self.cone_db = np.delete(self.cone_db, remove_index, axis=0 )
                     
        else:
            for c in cones:
                new_cone = np.array([c[0],c[1], self.R_max_cov, round(c[3]) ,self.add_hit])
                if self.cone_db.shape[0] > 0:
                    self.cone_db = np.vstack((self.cone_db,new_cone))
                else:
                    self.cone_db = new_cone.reshape(1,5)

        self.transformed_cones = np.array([])

        # for r in self.cone_db:
        #     print(np.around(r[0],2), np.around(r[1],2), np.around(r[2],4), int(r[3]), int(r[4]))
        
        # print("-------------")

        rviz_db = np.array([])

        if self.cone_db.shape[0] > 0:
            rviz_db = copy.deepcopy(self.cone_db)
            rviz_db = np.hstack((rviz_db, np.zeros((rviz_db.shape[0], 1)) ))
            try:
                rviz_db[db_in_fov_index, 5] = 1
            except:
                rviz_db[0, 5] = 1

        self.output_4_rviz(rviz_db.astype(np.float32))


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
