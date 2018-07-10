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
        self.angle_FOV = 30
        
        self.transformed_cones = []

        self.legit_cone_hits  = 20
        self.freq_clean_db = 1

        # -----

        self.main()

    def callback_odom(self, msg):
        self.last_odom = msg

    def callback_reactive(self, msg):
        last_reactive = msg.data.reshape(int(msg.data.shape[0]/4),4)

        self.listener.waitForTransform('zed_left_camera','map', rospy.Time(0), rospy.Duration(1.0))
        self.listener.lookupTransform('zed_left_camera','map', rospy.Time(0))

        transformed_cones = np.array([])
        self.no_transformed_cones = last_reactive

        for cone in last_reactive:
            conePoint_local = PointStamped()
            conePoint_local.header.frame_id = 'zed_center'
            conePoint_local.point.x = cone[0]
            conePoint_local.point.y = cone[1] 
            #conePoint_local.point.z = -cone[1]
            conePoint_global = PointStamped()
            conePoint_global = self.listener.transformPoint("/map", conePoint_local)

            global_cone = np.array([conePoint_global.point.x, conePoint_global.point.y, float(cone[3])], dtype=np.float32).reshape(1,3)

            if transformed_cones.shape[0] > 0:
                transformed_cones = np.vstack((transformed_cones, global_cone))
            else:
                transformed_cones = global_cone

        self.transformed_cones = transformed_cones

    def update_cone_db(self, event):

        cones = copy.deepcopy(self.transformed_cones)
        odom = copy.copy(self.last_odom)
        odom.pose.pose.position.y = odom.pose.pose.position.y + self.bias_left_lense

        db_in_fov_index = []

        # Compute FOV

        if self.cone_db.shape[0] > 0:

            # position = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y]).reshape(1,2)
            # (r, p, y) = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
            # orientation = y * 180 / np.pi * (-1)

            # max_angle_FOV = orientation + self.angle_FOV
            # min_angle_FOV = orientation - self.angle_FOV

            # if max_angle_FOV >= 180:
            #     max_angle_FOV = max_angle_FOV - 360
            
            # if min_angle_FOV <= -180:
            #     min_angle_FOV = min_angle_FOV + 360

            # print("max: "+str(max_angle_FOV)+"min: "+str(min_angle_FOV))            

            # dist = np.linalg.norm(self.cone_db[:,0:2]-position ,axis=1)

            # db_in_dist_index = np.argwhere(dist <= self.dist_FOV)

            # if db_in_dist_index.shape[0] > 0:
            #     cones_in_dist = self.cone_db[db_in_dist_index.tolist(),0:2]
            #     cones_in_dist = cones_in_dist.reshape(cones_in_dist.shape[0],2)
            #     cones_angles = np.arctan2(cones_in_dist[:,1], cones_in_dist[:,0]) * 180 / np.pi

            #     db_in_fov_index = np.argwhere( np.logical_and((cones_angles-orientation) < max_angle_FOV , (cones_angles-orientation) > min_angle_FOV) )
            #     db_in_fov_index = db_in_fov_index.squeeze()

            #     db_in_fov = copy.copy(self.cone_db[db_in_fov_index, :])
            #     try:
            #         db_in_fov = db_in_fov.reshape(db_in_fov_index.shape[0],self.cone_db.shape[1])
            #     except:
            #         db_in_fov = db_in_fov.reshape(1,self.cone_db.shape[1])
                

                # Checked if associate


            # Check cones on db which are also in the FOV
            db_in_fov_index = []
            self.listener.waitForTransform('zed_left_camera','map', rospy.Time(0), rospy.Duration(1.0))
            self.listener.lookupTransform('zed_left_camera','map', rospy.Time(0))
            for i in range(self.cone_db.shape[0]):
                c = self.cone_db[i,:]
                conePoint_global = PointStamped()
                conePoint_global.header.frame_id = '/map'
                conePoint_global.point.x = c[0]
                conePoint_global.point.y = c[1] 
                conePoint_local = PointStamped()
                conePoint_local = self.listener.transformPoint("/zed_left_camera", conePoint_global)

                angle = (np.arctan2(conePoint_local.point.y, conePoint_local.point.x) * 180 / np.pi) 
                dist = np.sqrt(conePoint_local.point.y**2 + conePoint_local.point.x**2)

                # print("Global cones: "+str(angle))

                if dist <= self.dist_FOV and angle <= self.angle_FOV and angle >= -self.angle_FOV:
                    db_in_fov_index.append(i)

            db_in_fov = copy.deepcopy(self.cone_db[db_in_fov_index])
            db_in_fov_index = np.array(db_in_fov_index)
            # Associated column
            ass_column = np.zeros((db_in_fov.shape[0], 1))
            db_in_fov = np.hstack((db_in_fov, ass_column))

            # self.listener.waitForTransform('zed_left_camera','map', rospy.Time(0), rospy.Duration(1.0))
            # self.listener.lookupTransform('zed_left_camera','map', rospy.Time(0))
        
            for ic in range(cones.shape[0]):
                c = cones[ic]
                # Reactive cone in the FOV
                
                reactive_angle = (np.arctan2(self.no_transformed_cones[ic,1], self.no_transformed_cones[ic,0]) * 180 / np.pi) 
                reactive_dist = np.sqrt(self.no_transformed_cones[ic,1]**2 + self.no_transformed_cones[ic,0]**2)

                # print("Angle:"+str(reactive_angle))

                if reactive_dist > self.dist_FOV or reactive_angle > self.angle_FOV or reactive_angle < -self.angle_FOV:
                    continue

                print("In FOV !")

                # Data association
                dist_db = np.linalg.norm(db_in_fov[:,0:2]-c[0:2] ,axis=1)
                if dist_db.shape[0] > 0 and np.amin(dist_db) <= self.R_max_cov:
                    i = np.argmin(dist_db)
                    # Moving Avg.
                    db_in_fov[i,0:2] = self.alpha * db_in_fov[i,0:2] + (1-self.alpha) * c[0:2]
                    # Color
                    db_in_fov[i,3] = c[2]
                    # Hits
                    db_in_fov[i,4] = min(db_in_fov[i,4]+self.add_hit, self.max_hits)
                    # Covariance
                    db_in_fov[i,2] = max(self.R_max_cov - self.delta_cov * db_in_fov[i,4], self.R_min_cov)
                    # Associated
                    db_in_fov[i,5] = 1

                else:
                    new_cone = np.array([c[0],c[1], self.R_max_cov, c[2],self.add_hit])
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
                new_cone = np.array([c[0],c[1], self.R_max_cov, c[2],self.add_hit])
                if self.cone_db.shape[0] > 0:
                    self.cone_db = np.vstack((self.cone_db,new_cone))
                else:
                    self.cone_db = new_cone.reshape(1,5)

        self.transformed_cones = np.array([])

        for r in self.cone_db:
            print(np.around(r[0],2), np.around(r[1],2), np.around(r[2],4), int(r[3]), int(r[4]))
        
        print("-------------")

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
        


    def output_4_nav(self, event): 

        # the_chosen_ones = np.argwhere(self.cone_db[:,3] >= self.threshold_hits)
        # output_array = self.cone_db[the_chosen_ones,0:3]
        print()
        # for r in self.cone_db:
        #     print(np.around(r[0],2), np.around(r[1],2), np.around(r[2],4), int(r[3]), int(r[4]))
        
        # print("-------------")

    def clean_cone_db(self, event): 
        legit_cones_index = np.argwhere(self.cone_db[:,4] >= self.legit_cone_hits)
        legit_cones = self.cone_db[legit_cones_index,:]

        for legit in legit_cones:
            dist = np.linalg.norm(self.cone_db[:,0:2]-legit[0:2] ,axis=1)
            ghosts_index = np.argwhere(dist <= self.R_max_cov)
            ghosts_index = np.delete(ghosts_index, np.argmax(ghosts_index[:,4]), axis=0 )
            self.cone_db = np.delete(self.cone_db, ghosts_index, axis=0 )
            


    def main(self):
        rospy.init_node('mapping_node')
        rospy.Subscriber('/reactive_cones', numpy_msg(Floats), self.callback_reactive)
        rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom)
        # rospy.Timer(rospy.Duration(1.0/self.freq_output), self.output_4_nav)
        rospy.Timer(rospy.Duration(1.0/self.freq_update_db), self.update_cone_db)
        rospy.Timer(rospy.Duration(1.0/self.freq_clean_db), self.clean_cone_db)
        rospy.spin()

Global_Mapping_Node()
