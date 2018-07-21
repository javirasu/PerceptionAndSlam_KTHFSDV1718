#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import copy

class Reactive_Mapping_Node():

    def __init__(self):
        # Global variables
        self.acc_frames = 3
        self.perc_cluster = 0.65
        self.rad_cluster = 0.5

        self.bias_left_lense = 0.06

        self.delta = 99/1e4

        # -----

        self.integrated = np.array([])
        self.int_counter = 0

        # -----

        self.main()

    def callback_perception(self, msg):
        last_frame_left = msg.data.reshape(int(msg.data.shape[0]/4),4)
        last_frame = copy.copy(last_frame_left)
        last_frame[:,1] = last_frame[:,1] + float(self.bias_left_lense)
        print(last_frame)
        
        if self.integrated.shape[0] == 0:
            self.integrated = last_frame
        else:
            self.integrated = np.vstack((self.integrated, last_frame))
        
        self.int_counter = self.int_counter + 1

        if self.int_counter == self.acc_frames:
            centroids = self.cluster_frames()
            self.publish_reactive_map(centroids)
            self.int_counter = 0
            self.integrated = np.array([])

    def cluster_frames(self):
        
        points = self.integrated[:,0:2]

        dist_matrix = np.around(np.linalg.norm(points - points[:,None], axis = -1) , decimals=3)

        cluster_id = np.zeros((points.shape[0],1), dtype=int)

        for i in range(points.shape[0]):
            neighbours = np.argwhere(dist_matrix[i,:] < self.rad_cluster)

            if cluster_id[i] == 0:
                cluster_id[i] = np.amax(cluster_id)+1      

                for n in neighbours:
                    if cluster_id[n] == 0:
                        cluster_id[n] = cluster_id[i]

            else:
                
                for n in neighbours:
                    if cluster_id[n] == 0:
                        cluster_id[n] = cluster_id[i]
              
        unique, counts = np.unique(cluster_id, return_counts=True)
        cluster_dict = dict(zip(unique,counts))

        cluster_centroid = []

        for k in cluster_dict.keys():
            n_points = cluster_dict[k]
            if n_points >= int(round(self.acc_frames * self.perc_cluster)):
                indexes = np.argwhere(cluster_id==k)[:,0]
                p_clusters = points[indexes,:]
                cent = np.mean(p_clusters, axis=0)
                covar = np.var(p_clusters, axis=0)
                covar = np.sqrt(covar[0]**2+covar[1]**2)
                color_points = self.integrated[indexes,3]
                uniq, count = np.unique(color_points, return_counts=True)
                color = uniq[np.argmax(count)]
                cent = np.hstack((cent, covar, color))
                cluster_centroid.append(cent)

        return cluster_centroid

    def publish_reactive_map(self, reactive):
        reactive = np.array(reactive, dtype=np.float32)
        cone_publisher = rospy.Publisher('/reactive_cones', numpy_msg(Floats), queue_size=10)
        cone_publisher.publish(reactive.flatten())
        for r in reactive:
            print(np.around(r[0],2), np.around(r[1],2), np.around(r[2],4), int(r[3]))
        print("----------")


    def main(self):
        rospy.init_node('reactive_mapping_node')
        rospy.Subscriber('/perception_cones', numpy_msg(Floats), self.callback_perception)
        rospy.spin()

Reactive_Mapping_Node()
