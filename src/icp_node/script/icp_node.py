#!/usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import copy
from ICP import ICP_pose_estimation
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class ICP_Node():
    
    def __init__(self):

        self.n_meas = 10
        self.buffer_meas = np.zeros((self.n_meas, 6))
        self.pointer = 0
        self.full_buffer = False

        self.main()

    def callback_mapping(self, msg):
        '''
        Expected input for the node:
            np.array( [pose]
                    [cone_db]
                    [cone_reactive]
                    [cone_db]
                    [cone_reactive]
                    [cone_db]
                    [...])
        '''
        data = msg.data.reshape(int(msg.data.shape[0]/3),3)
        
        pose = np.array([])
        db_cones = np.array([])
        reactive_cones = np.array([])

        for i in range(data.shape[0]):

            if i == 0:
                pose = copy.deepcopy(data[0,:])
            elif i % 2:
                if reactive_cones.shape[0] > 0:
                    reactive_cones = np.hstack((reactive_cones, data[i,0:2].T ))
                else:
                    reactive_cones = data[i,0:2].T
            else:
                if db_cones.shape[0] > 0:
                    db_cones = np.hstack((db_cones, data[i,0:2].T ))
                else:
                    db_cones = data[i,0:2].T

        try:
            self.estimate_pose_and_publish(pose,db_cones, reactive_cones)
        except:
            return

    def estimate_pose_and_publish(self,pose,db,react):

        estimated_pose = ICP_pose_estimation(react, db, pose)
        meas_pose = np.array([estimated_pose[0],estimated_pose[1],0,0,0,estimated_pose[2]]).reshape(1,6)

        self.buffer_meas[self.pointer,:] = meas_pose
        self.pointer = self.pointer + 1
        if self.pointer == self.n_meas:
            self.full_buffer = True
            self.pointer = 0
        
        if self.full_buffer:
            cov = np.cov(self.buffer_meas.T).flatten() 

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.get_rostime()

            msg.pose.pose.position.x = estimated_pose[0]
            msg.pose.pose.position.y = estimated_pose[1]
            msg.pose.pose.position.z = 0
            
            msg.pose.pose.orientation =  tf.transformations.quaternion_from_euler(0,0,estimated_pose[2])
            msg.pose.covariance = cov.tolist()

            pub_imu = rospy.Publisher('/icp_estimate', PoseWithCovarianceStamped, queue_size=10)
            pub_imu.publish(msg)



    def main(self):
        rospy.init_node('icp_node')
        rospy.Subscriber('/icp_input', numpy_msg(Floats), self.callback_mapping)
        rospy.spin()

ICP_Node()