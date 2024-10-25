#!/usr/bin/env python
import numpy as np
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import time
import message_filters
from std_msgs.msg import UInt16
from std_msgs.msg import Int32MultiArray
import cv2
#np.set_printoptions(threshold=sys.maxsize)


class ImageListener:
    def __init__(self, rgb_topic, depth_topic):
        print("INIT")
        start_loading_model = time.time() 
        self.model = YOLO("/home/dronelab/Documents/thesis/ba/code/catkin_ws/src/drone_inspection_jetson/src/weights/30_06_2023_license_plate.pt") 
        print("Model nach "+ str(time.time() - start_loading_model) + " sekunden geladen")
        
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.bridge = CvBridge()
        rgb_sub = message_filters.Subscriber(rgb_topic, msg_Image)
        depth_sub = message_filters.Subscriber(depth_topic, msg_Image)
        ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
        ts.registerCallback(self.image_bounding_boxes)
        
        #self.pub_depth_int = rospy.Publisher('depth_int', UInt16, queue_size=2)
        self.pub_bouding_box = rospy.Publisher('bounding_boxes', Int32MultiArray, queue_size=2)  #Bounding Boxes vom Typ [x_min, y_min, x_max, y_max, x_center, y_center,depth_to_center]
        self.pub_bb_image = rospy.Publisher('bb_license_plate_image', msg_Image, queue_size=2)  #Bounding Boxes vom Typ [x_min, y_min, x_max, y_max, x_center, y_center,depth_to_center]

    def plot_results(self, result):
        res_plotted = result[0].plot()
        return res_plotted


    def detect_img(self, img):
        result = self.model(img, verbose=False, conf = 0.6)
        return result

    def get_center_of_box(self, box):
        if len(box) == 0: 
            return 0,0,0,0,0,0
        box = box[0]
        x_min = box[0].item()
        y_min = box[1].item()
        x_max = box[2].item()
        y_max = box[3].item()

        x_center = int(x_min+(x_max-x_min)/2)
        y_center = int(y_min+(y_max-y_min)/2)

      

        #print("bonding_box: ", box)
        #print("x_center: ", x_center)
        #print("y_center: ", y_center)

        
        return int(x_min), int(y_min), int(x_max), int(y_max), x_center,y_center

    def get_depth_data(self, aligned_depth_frame, x_center, y_center):
        #depth = np.asanyarray(aligned_depth_frame.get_data())
        if x_center == y_center == 0:
            return 0
        #print("ALLIGNED DEPTH FRAME:", aligned_depth_frame)
        depth = aligned_depth_frame[y_center,x_center].astype(float)    #Setzt voraus, dass RGB und Depth Image von gleicher Dimension sind!!!
        if np.isnan(depth):
            return 0
        print("DEPTH", depth)

        #depth = depth * 1000 #für Gazebo Simulation
        print(depth)
        print("depth before scale", depth)      
        return int(depth)

    def image_bounding_boxes(self, rgb_sub, depth_sub):
        try:
            #print("start: ", time.time())
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_sub, rgb_sub.encoding)
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_sub, depth_sub.encoding)

            #print("\n")
            #print(('Dimesnionen rgb image', cv_rgb_image.shape))

            #print(('Dimesnionen depth image', cv_depth_image.shape))
            a = time.time()
            detected_img = self.detect_img(cv_rgb_image)
            #print("detected_img: ", detected_img[0].boxes.xyxy)
            #print("Es dauerte " + str(time.time()-a) + " sekunden für yolo um den frame zu detecten")
            result_img = self.plot_results(detected_img)

            bounding_boxes = detected_img[0].boxes.xyxy
            print("bounding_boxes: ", bounding_boxes)
            x_min, y_min, x_max, y_max, x_center, y_center = self.get_center_of_box(bounding_boxes)
            result_img = cv2.circle(result_img, (x_center,y_center), radius=10, color=(0, 0, 255), thickness=-1)  #Nur für Sim Visualisierung
            self.pub_bb_image.publish(self.bridge.cv2_to_imgmsg(result_img, encoding='rgb8'))  ###Bei Jetson Verion herausnehmen (nur zu visualisierungszwecken)

            depth = self.get_depth_data(cv_depth_image, x_center, y_center)

            #self.pub_depth_int.publish(depth)
            array_msg = Int32MultiArray()
            #print([type(x_min), type(y_min), type(x_max), type(y_max), type(x_center), type(y_center)])
            array_msg.data = [x_min, y_min, x_max, y_max, x_center, y_center, depth]  # Example array values
            self.pub_bouding_box.publish(array_msg)
            #print("end: ", time.time())


        except CvBridgeError as e:
            print(e)
            return



if __name__ == '__main__':
    rospy.init_node("image_bounding_boxes")
    #rgb_topic = '/camera/color/image_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
    rgb_topic = '/d400/color/image_raw'
    #depth_topic = '/camera/aligned_depth_to_color/image_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
    depth_topic = '/d400/aligned_depth_to_color/image_raw'
    listener = ImageListener(rgb_topic, depth_topic)
    rospy.spin()
