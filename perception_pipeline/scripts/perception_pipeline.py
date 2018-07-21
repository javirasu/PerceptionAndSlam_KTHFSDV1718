#!/usr/bin/env python
'''
    Author -    Javier Rasines (javierrs@kth.se)
                Ajinkya Khoche (khoche@kth.se)
    ------
	Description
	-----------
		This program subscribes to:
            - Zed camera image
            - Zed camera point cloud and (possibly)
            - Zed odometry
		It takes a zed camera image, passes it through SSD mobilenet
        object detection to detect cones and then creates a local map

        This program publishes:
            - Processed Zed image with bounding Boxes and depth on cones
            - Local Map: Its a numpy array with following structure:
            [x_w, y_w, z_w, color] where x_w is world coordinates of cones
    
    NOTE: Color scheme for cones is: 
            0-  YELLOW
            1-  BLUE
            2-  ORANGE
            3-  WHITE
            4-  BLACK

==========================================================================
	History
	-------
		Version No.			Date			Author
		------------------------------------------
		1.x					2018/06/10		Javier, Ajinkya


	Revision Description
	--------------------
		1.x		---	STEPS
'''
import cv2
import numpy as np
import tensorflow as tflow
from matplotlib import pyplot as plt
from utils import visualization_utils as vis_util
from utils import label_map_util
from ConeDetection import *
#from cone_img_processing2 import *
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image , PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import time
import copy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import PointStamped
import tf
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

#Intrinsic camera parameters
cy = 352.49
fy = 672.55
cx = 635.18
fx = 672.55

# Set threshold for detection of cone for object detector
threshold_cone = 0.5

#cap = cv2.VideoCapture(1)

#Set path to check point and label map
#PATH_TO_CKPT = './frozen_orange_net.pb'
PATH_TO_CKPT = os.path.dirname(os.path.realpath(__file__)) + '/frozen_cone_graph.pb'  
PATH_TO_LABELS = os.path.dirname(os.path.realpath(__file__)) + '/label_map.pbtxt'

#Define no, of classes
NUM_CLASSES = 1         #only one class, i.e. cone

## Load a (frozen) Tensorflow model into memory.
detection_graph = tflow.Graph()
with detection_graph.as_default():
  od_graph_def = tflow.GraphDef()
  with tflow.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tflow.import_graph_def(od_graph_def, name='')

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# def load_image_into_numpy_array(image):
#   (im_width, im_height) = image.size
#   return np.array(image.getdata()).reshape(
#       (im_height, im_width, 3)).astype(np.uint8)

class Frame:
    def __init__(self):
        self.image = []
        self.flagImage = False
        self.depth = []
        self.flagDepth = False
        self.odom_x = 0
        self.odom_y = 0
        self.odom_z = 0

# create a global object of Frame
lastFrame = Frame()

class Cone:
    def __init__(self):
        self.color = 0
        self.x = 0
        self.y = 0

# Creates Rviz markers and publishes it
def publish_local_map(local_map):
    marker_publisher = rospy.Publisher('/local_map_markers', Marker, queue_size=10)
    # global listener
    # listener.lookupTransform('zed_left_camera','map', rospy.Time(0))

    i = 0
    for cone in local_map:
        marker = Marker()
        marker.id = i
        # marker.id = int(np.random.randint(1)*1000)
        # marker.header.frame_id = "/zed_left_camera"
        marker.header.frame_id = "map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
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
        marker.lifetime = rospy.Duration(3)
        marker.pose.orientation.w = 1.0
        # marker.pose.position.x = round(cone[2],1)
        # marker.pose.position.y = round(-cone[0],1)
        # marker.pose.position.z = round(-cone[1],1)
        conePoint_local = PointStamped()
        conePoint_local.header.frame_id = 'zed_left_camera'
        conePoint_local.point.x = cone[2]
        conePoint_local.point.y = -cone[0]
        conePoint_local.point.z = -cone[1]

        conePoint_global = PointStamped()
        # conePoint_global = listener.transformPoint("/map", conePoint_local)

        marker.pose.position = conePoint_global.point

        # marker.pose.position.x = round(cone[2] + lastFrame.odom_x , 4)
        # marker.pose.position.y = round(-cone[0] + lastFrame.odom_y , 4)
        # marker.pose.position.z = round(-cone[1] + lastFrame.odom_z , 4)

        marker_publisher.publish(marker)
        rospy.Rate(100).sleep()
        i = i + 1
        
# Publish cones as numpy array with the cone types and positions
def publish_cones(local_map, transform_to_map=False):
    if local_map.shape[0] == 0:
        return

    cone_publisher = rospy.Publisher('/perception_cones', numpy_msg(Floats), queue_size=10)
    # global listener
    # listener.lookupTransform('zed_left_camera','map', rospy.Time(0))
    
    transformed_cones = np.array([])

    for cone in local_map:
        conePoint_local = PointStamped()
        conePoint_local.header.frame_id = 'zed_left_camera'
        # conePoint_local.point.x = cone[2]
        # conePoint_local.point.y = -cone[0]
        # conePoint_local.point.z = -cone[1]
        conePoint_local.point.x = cone[2]
        conePoint_local.point.y = -cone[0]
        conePoint_local.point.z = -cone[1]
        conePoint_global = PointStamped()
        # conePoint_global = listener.transformPoint("/map", conePoint_local)

        # global_cone = np.array([conePoint_global.point.x, conePoint_global.point.y, conePoint_global.point.z, float(cone[3])], dtype=np.float32).reshape(1,4)
        local_cone = np.array([conePoint_local.point.x, conePoint_local.point.y, conePoint_local.point.z, float(cone[3])], dtype=np.float32).reshape(1,4)

        if transform_to_map:
            # if transformed_cones.shape[0] > 0:
            #     transformed_cones = np.vstack((transformed_cones, global_cone))
            # else:
            #     transformed_cones = global_cone
            print()
        else:
            if transformed_cones.shape[0] > 0:
                transformed_cones = np.vstack((transformed_cones, local_cone))
            else:
                transformed_cones = local_cone
                
    cone_publisher.publish(transformed_cones.flatten())





# OLD call back. This function caused loading of tflow graph in every iteration and
# subsequently slowed down the FPS considerably. Hence, not used.
# def callback_detect(image_message):
#     tick = time.time()
#     bridge = CvBridge()
#     image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

#     with detection_graph.as_default():
#         with tflow.Session(graph=detection_graph) as sess:

#             image_np = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 

#             # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
#             image_np_expanded = np.expand_dims(image_np, axis=0)
#             image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
#             # Each box represents a part of the image where a particular object was detected.
#             boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
#             # Each score represent how level of confidence for each of the objects.
#             # Score is shown on the result image, together with the class label.
#             scores = detection_graph.get_tensor_by_name('detection_scores:0')
#             classes = detection_graph.get_tensor_by_name('detection_classes:0')
#             num_detections = detection_graph.get_tensor_by_name('num_detections:0')
#             # Actual detection.
#             (boxes, scores, classes, num_detections) = sess.run(
#                 [boxes, scores, classes, num_detections],
#                 feed_dict={image_tensor: image_np_expanded})
#             # Visualization of the results of a detection.

#             # Definition of boxes [ymin, xmin, ymax, xmax]
#             boxes = np.squeeze(boxes)
#             scores = np.squeeze(scores)

#             width = image_np.shape[1]
#             height = image_np.shape[0]
#             # width, height = cv2.GetSize(image_np)
#             output_img = image_np.copy()

#             center_x = []
#             center_y = []
#             center_z = []
#             center_color =[]

#             for i in range(boxes.shape[0]):

#                 if np.all(boxes[i] == 0) or scores[i] < threshold_cone:
#                     continue
                
#                 b = boxes[i]

#                 box_width = np.abs(float(b[3])-float(b[1]))
#                 box_height  = np.abs(float(b[2])-float(b[0]))

#                 x = int(b[1] * width)
#                 y = int(b[0] * height)
#                 h = int(box_height * height)
#                 w = int(box_width * width)

#                 candidate = image_np[y:y+h, x:x+w]
#                 y = y + 1

#                 result = detectCone1(candidate)

#                 center_x.append(round(x+w/2))
#                 center_y.append(round(y+h/2))
#                 depth = lastFrame.depth[round(y+h/2),round(x+w/2)]
                
#                 ##########################
#                 center_z.append(depth)
#                 center_color.append(result)

#                 print(result)

#                 if result == 0:
#                     print("Yellow Cone")
#                     cv2.rectangle(output_img, (int(b[1] * width),int(b[0] * height)), (x+w,y+h), (0, 255, 255), 7)
#                     cv2.putText(output_img, 'yellow cone', (int(b[1] * width),int(b[0] * height)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
#                 if result == 1:
#                     print("Blue Cone")
#                     cv2.rectangle(output_img, (int(b[1] * width),int(b[0] * height)), (x+w,y+h), (255, 0, 0), 7)
#                     cv2.putText(output_img, 'blue cone', (int(b[1] * width),int(b[0] * height)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
#                 if result == 2:
#                     print("Orange Cone")
#                     cv2.rectangle(output_img, (int(b[1] * width),int(b[0] * height)), (x+w,y+h), (0,165,255), 7)
#                     cv2.putText(output_img, 'orange cone', (int(b[1] * width),int(b[0] * height)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            
#             #print(output_img.shape)        
#             cv2.imshow('object detection', cv2.resize(output_img, (image_np.shape[1],image_np.shape[0])))
#             cv2.waitKey(1)

#             local_map = np.array(center_x,center_y,center_z,center_color)
            
#             tock = time.time() - tick
            
#             publish_local_map(local_map)

#             publish_cones(local_map)

#             #tock = time.time() - tick

#             print("FPS: "+str(1/tock))
#             print("Time: "+str(tock))

#call back function to just display image
def callback_show(image_message):
    tick = time.time()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    cv2.imshow('object detection', image)
    cv2.waitKey(1)
    tock = time.time() - tick
    print("FPS: "+str(1/tock))
    print("Time: "+str(tock))

#call back function to store image in class object 
def callback_storage_image(image_message):
    bridge = CvBridge()
    global lastFrame
    lastFrame.image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    #lastFrame.image = cv2.cvtColor(lastFrame.image, cv2.COLOR_BGR2RGB)
    lastFrame.flagImage = True

#call back function to store depth in class object
def callback_depth(image_message):
    bridge = CvBridge()
    global lastFrame
    lastFrame.depth = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    lastFrame.flagDepth = True

#call back function to store depth in class object
def callback_zedOdom(odom):
    position = odom.pose.pose.position
    global lastFrame
    lastFrame.odom_x = position.x
    lastFrame.odom_y = position.y
    lastFrame.odom_z = position.z

# MAIN    
def mainLoop():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/zed/left/image_rect_color", Image, callback_storage_image)
    rospy.Subscriber("/zed/depth/depth_registered", Image, callback_depth)
    rospy.Subscriber("/zed/odom", Odometry, callback_zedOdom)
    processedImage = rospy.Publisher("/processedImage", Image, queue_size=10)

    global listener
    listener = tf.TransformListener()
    
    r = rospy.Rate(100) # Hz
    with detection_graph.as_default():
        with tflow.Session(graph=detection_graph) as sess:
            while not rospy.is_shutdown():
                tick = time.time()
                global lastFrame
                processFrame = copy.deepcopy(lastFrame)

                if processFrame.flagImage == True and processFrame.flagDepth == True:
                    
                    # image_np = cv2.resize(processFrame.image, (0,0), fx=0.5, fy=0.5) 
                    image_np = processFrame.image
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                    # Each box represents a part of the image where a particular object was detected.
                    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Each score represent how level of confidence for each of the objects.
                    # Score is shown on the result image, together with the class label.
                    scores = detection_graph.get_tensor_by_name('detection_scores:0')
                    classes = detection_graph.get_tensor_by_name('detection_classes:0')
                    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                    # Actual detection.
                    (boxes, scores, classes, num_detections) = sess.run(
                        [boxes, scores, classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})
                    # Visualization of the results of a detection.

                    # Definition of boxes [ymin, xmin, ymax, xmax]
                    boxes = np.squeeze(boxes)
                    scores = np.squeeze(scores)

                    width = image_np.shape[1]
                    height = image_np.shape[0]
                    # width, height = cv2.GetSize(image_np)
                    #image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
                    output_img = image_np.copy()
                    

                    center_x = []
                    center_y = []
                    center_z = []
                    center_color =[]

                    for i in range(boxes.shape[0]):

                        if np.all(boxes[i] == 0) or scores[i] < threshold_cone:
                            continue
                        
                        b = boxes[i]

                        box_width = np.abs(float(b[3])-float(b[1]))
                        box_height  = np.abs(float(b[2])-float(b[0]))

                        x = int(b[1] * width)
                        y = int(b[0] * height)
                        h = int(box_height * height)
                        w = int(box_width * width)

                        candidate = image_np[y:y+h, x:x+w]
                        y = y + 1

                        result = detectCone1(candidate)

                        # z = lastFrame.depth[int(y+h/2),int(x+w/2)]
                        # Taking 10 pixel square
                        square_size = 5
                        depth_square = lastFrame.depth[int(-square_size+round(y+h/2)):int(square_size+round(y+h/2)),int(-square_size+round(x+w/2)):int(square_size+round(x+w/2))]
                        depth_square = copy.copy(depth_square)
                        bad_I = np.argwhere(np.isnan(depth_square))
                        depth_square[bad_I[:,0],bad_I[:,1]] = 0
                        bad_I = np.argwhere(np.isinf(depth_square))
                        depth_square[bad_I[:,0],bad_I[:,1]] = 0

                        valid_index = np.nonzero(depth_square)
                        if valid_index[0].shape[0] == 0 or valid_index[1].shape[0] == 0:
                            continue

                        z = np.mean(depth_square[valid_index[0], valid_index[1]])

                        # z = lastFrame.depth[int(y+h/2),int(x+w/2)]
                            
                        # use intrinsic camera parameters to convert from pixel coordinate to 
                        # world coordinate (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html)
                        try:
                            y_w = (round(y+h/2) - cy) / fy * z
                            x_w = (round(x+w/2) - cx) / fx * z
                        except:
                            continue

                        center_x.append(x_w)
                        center_y.append(y_w)
                        center_z.append(z)
                        center_color.append(result)

                        # print(result)

                        if result == 0:
                            print("Yellow Cone")
                            cv2.rectangle(output_img, (int(b[1] * width),int(b[0] * height)), (x+w,y+h), (0, 255, 255), 7)
                            cv2.putText(output_img, 'yellow cone', (int(b[1] * width),int(b[0] * height)-30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                            cv2.putText(output_img,  str(round(z,3))+" m", (int(b[1] * width),int(b[0] * height)-5), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)

                        if result == 1:
                            print("Blue Cone")
                            cv2.rectangle(output_img, (int(b[1] * width),int(b[0] * height)), (x+w,y+h), (255, 0, 0), 7)
                            cv2.putText(output_img, 'blue cone', (int(b[1] * width),int(b[0] * height)-30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                            cv2.putText(output_img,  str(round(z,3))+" m", (int(b[1] * width),int(b[0] * height)-5), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                        if result == 2:
                            print("Orange Cone")
                            cv2.rectangle(output_img, (int(b[1] * width),int(b[0] * height)), (x+w,y+h), (0,165,255), 7)
                            cv2.putText(output_img, 'orange cone', (int(b[1] * width),int(b[0] * height)-30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
                            cv2.putText(output_img,  str(round(z,3))+" m", (int(b[1] * width),int(b[0] * height)-5), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)

                    #cv2.imshow('object detection', cv2.resize(output_img, (image_np.shape[1],image_np.shape[0])))
                    #cv2.waitKey(1)
                    
                    processedImage.publish(CvBridge().cv2_to_imgmsg(output_img))
                    
                    r.sleep()

                    local_map = np.array((center_x,center_y,center_z,center_color)).T
                   
                    publish_local_map(local_map)

                    publish_cones(local_map)

                    tock = time.time() - tick

                    # print("FPS: "+str(1/tock))
                    # print("Time: "+str(tock))
                    processFrame.flagImage = False
                    processFrame.flagPointCloud = False

        r.sleep()


if __name__ == '__main__':
    mainLoop()
    
