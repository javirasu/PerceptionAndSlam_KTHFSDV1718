#!/usr/bin/env python
import rospy
import copy
from sensor_msgs.msg import Imu
import numpy as np
import tf

class IMU_fix():

    def __init__(self):

        self.n_meas = 10
        self.buffer_ori = np.zeros((self.n_meas, 3))
        self.buffer_vel = np.zeros((self.n_meas, 3))
        self.buffer_acc = np.zeros((self.n_meas, 3))
        self.pointer = 0
        self.full_buffer = False

        self.main()

    def callback_imu(self, msg):

        msg = copy.deepcopy(msg)
        msg.header.frame_id = "imu_link"
        msg.header.stamp = rospy.get_rostime()

        # (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # meas_ori = np.array([r,p,y])
        # meas_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        # meas_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # self.buffer_ori[self.pointer,:] = meas_ori
        # self.buffer_vel[self.pointer,:] = meas_vel
        # self.buffer_acc[self.pointer,:] = meas_acc
        # self.pointer = self.pointer + 1
        # if self.pointer == self.n_meas:
        #     self.full_buffer = True
        #     self.pointer = 0

        
        # if self.full_buffer:

        #     cov_ori = np.cov(self.buffer_ori.T).flatten() 
        #     cov_vel = np.cov(self.buffer_vel.T).flatten() 
        #     cov_acc = np.cov(self.buffer_acc.T).flatten() 

            # msg.orientation_covariance = cov_ori.tolist()
            # msg.angular_velocity_covariance = cov_vel.tolist()
            # msg.linear_acceleration_covariance = cov_acc.tolist()

        pub_imu = rospy.Publisher('/imu_fixed', Imu, queue_size=10)
        pub_imu.publish(msg)

    def main(self):
        rospy.init_node('imu_fix_node')
        rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        rospy.spin()

IMU_fix()