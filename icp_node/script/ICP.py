#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
from sklearn.metrics import mean_squared_error

def icp(a, b, init_pose=(0,0,0), no_iterations = 10):
    '''
    The Iterative Closest Point estimator.
    Takes two cloudpoints a[x,y], b[x,y], an initial estimation of
    their relative pose and the number of iterations
    Returns the affine transform that transforms
    the cloudpoint a to the cloudpoint b.
    Early stopping based on MSE implemented. As soon as it starts 
    to increase, the algorithm is finished.
    '''

    src = np.array([a.T], copy=True).astype(np.float32)
    dst = np.array([b.T], copy=True).astype(np.float32)

    #Initialise with the initial pose estimation
    Tr = np.array([[np.cos(init_pose[2]),-np.sin(init_pose[2]),init_pose[0]],
                   [np.sin(init_pose[2]), np.cos(init_pose[2]),init_pose[1]],
                   [0,                    0,                   1          ]])

    src = cv2.transform(src, Tr[0:2])

    mse_prev = 1e10

    for i in range(no_iterations):
        #Find the nearest neighbours between the current source and the destination cloudpoint
        # nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(dst[0])
        # distances, indices = nbrs.kneighbors(src[0])
        indices = np.arange(0,src.shape[1],dtype=np.int64).reshape(src.shape[1],1)

        #Compute the transformation between the current source and destination cloudpoint
        T = cv2.estimateRigidTransform(src, dst[0, indices.T], False)
        #Transform the previous source and update the current source cloudpoint
        src = cv2.transform(src, T)
        #Save the transformation from the actual source cloudpoint to the destination
        Tr = np.dot(Tr, np.vstack((T,[0,0,1])))

        src_mse = copy.deepcopy(src).squeeze()
        dst_mse = copy.deepcopy(dst).squeeze()

        mse_actual = mean_squared_error(src_mse, dst_mse)
        
        if mse_prev < mse_actual:
            break
        else:
            mse_prev = mse_actual

    return Tr[0:2]

def ICP_pose_estimation(react, db_cones, pose, plot_result = False):
    '''
    From the reactive, db cones and the current pose, a 
    new pose estimation will be computed.

    input:
        reactive_cones : np.array(2, X)  --> [x , y]
        db_cones       : np.array(2, X)  --> [x , y]
        pose           : np.array(1, 3)  --> [x , y , theta]

    output: [estimated_pose, covariance]
    '''
    # Compute affine transformation using ICP algorithm.
    # db cones are use as source, and reactive cones are destination
    M = icp(db_cones, react, init_pose=(0,0,0))

    # Compute MSE of the final transformation
    src = np.array([db.T]).astype(np.float32)
    res = cv2.transform(src, M)
    result = copy.deepcopy(res).squeeze().T

    mse = mean_squared_error(react, result)

    # The estimation is divided into translation and rotation.
    # To decomponse into R,T,S matrices: https://math.stackexchange.com/questions/237369/given-this-transformation-matrix-how-do-i-decompose-it-into-translation-rotati
    T = M[:,2].T
    R_matrix = M[0:2,0:2]
    sx = np.sqrt(R_matrix[0,0]**2 + R_matrix[0,1]**2)
    sy = np.sqrt(R_matrix[1,0]**2 + R_matrix[1,1]**2)
    theta1 = np.arccos(M[0,0]/sx)
    theta2 = np.arcsin(M[1,0]/sy) 

    if np.abs(theta1-theta2) < 1e4:
        # Check sign transform
        theta = theta1 * 180 / np.pi

    estimated_pose = np.array((pose[0]+T[0], pose[1]+T[1], pose[2]+theta)).reshape(1,3)

    if plot_result:
        print(mse)
        print(estimated_pose)

        plt.figure()
        plt.plot(db_cones[0],db_cones[1] , 'b.', label="db")
        plt.plot(res[0].T[0], res[0].T[1], 'r.', label="res")
        plt.plot(react[0], react[1] , 'g.', label="reactive")
        plt.legend()
        plt.show()

    return estimated_pose


TEST = True

if TEST:
    db_A = [1.0, 1.0]
    # db_B = [1.0, 2.0]
    # db_C = [2.0, 1.0]
    db_D = [2.0, 2.0]

    # db = np.array((db_A, db_B, db_C, db_D)).T
    db = np.array((db_A, db_D)).T
    # reactive = db + np.ones(db.shape) * 0.25 + np.random.normal(size=db.shape) * 0.1
    reactive = db + np.random.normal(loc=0.0,scale=0.1,size=db.shape)

    pose = np.array([0,0,0])

    ICP_pose_estimation(reactive,db,pose,plot_result=True)