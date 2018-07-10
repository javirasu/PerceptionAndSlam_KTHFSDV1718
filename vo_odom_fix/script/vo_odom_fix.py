#!/usr/bin/env python
import rospy
import copy
from nav_msgs.msg import Odometry
import numpy as np
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

class vo_odom_fix():

    def __init__(self):

        self.n_meas = 10
        self.buffer_meas = np.zeros((self.n_meas, 6))
        self.pointer = 0
        self.full_buffer = False

        self.main()

    def callback_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # (r, p, y) = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)

        meas = np.array([x,y,z,r,p,y])

        self.buffer_meas[self.pointer,:] = meas
        self.pointer = self.pointer + 1
        if self.pointer == self.n_meas:
            self.full_buffer = True
            self.pointer = 0
        
        if self.full_buffer:

            A = np.cov(self.buffer_meas.T)
            for a in A:
                print(a)
            print("------------")
            
            cov_mat = np.cov(self.buffer_meas.T).flatten()

            output = Odometry()

            output.header.frame_id = 'map'
            output.child_frame_id = 'zed_center'
            output.header.stamp = rospy.get_rostime()

            output.pose.pose = copy.deepcopy(msg.pose.pose)
            output.pose.covariance = cov_mat.tolist()

            pub_vo = rospy.Publisher('/vo_fixed', Odometry, queue_size=10)
            pub_vo.publish(output)



    def main(self):
        rospy.init_node('vo_odom_fix_node')
        rospy.Subscriber('/zed/odom', Odometry, self.callback_odom)
        rospy.spin()

vo_odom_fix()