#!/usr/bin/env python3
import numpy as np
from ultralytics import YOLO
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import time
from std_msgs.msg import UInt16, Int32MultiArray
from std_srvs.srv import Empty
import cv2
from smoke_project.srv import requestImage


class ImageListener:
    def __init__(self):
        start_loading_model = time.time() 
        rospack = rospkg.RosPack()
        # The type of the detection structure, 'service' : If you want a service-client structure, 'real-time' : publisher subscriber strucure with real time detection.
        self.detection_type = rospy.get_param('detection_type', 'service')
        self.model = YOLO('/home/dronelab/best.pt')
        #self.model = YOLO(rospack.get_path('smoke_project')+'/src/SensorRecognition/best.pt') 
        print("Model nach "+ str(time.time() - start_loading_model) + " sekunden geladen")

        #self.rgb_topic = '/usb_cam/image_raw'
        #self.rgb_topic = '/camera/color/image_raw'  # check the depth image topic in your Gazebo environmemt and replace this with your
        self.rgb_topic = '/d435/color/image_raw'
        self.rbg_data = Image()

        self.bridge = CvBridge()
        self.rgb_sub = rospy.Subscriber(self.rgb_topic,Image,self.imageCallback)

        
        self.pub_bb_image = rospy.Publisher('bb_smoke_sensor_image', Image, queue_size=2)  #Bounding Boxes vom Typ [x_min, y_min, x_max, y_max, x_center, y_center,height, width]
        if self.detection_type == 'service':
            self.bb_service = rospy.Service('/smoke_detection',requestImage, self.imageService)
        else:
            self.bb_pub = rospy.Publisher('/smoke_sensor_bb', Int32MultiArray, queue_size=2)
            self.rate = rospy.Rate(10)

        

    def imageCallback(self,data):
        self.rbg_data = data



    def detect_img(self, img):
        result = self.model(img, verbose=False, conf = 0.6)
        return result[0]

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


        
        return int(x_min), int(y_min), int(x_max), int(y_max), x_center,y_center


    def imageService(self,req):
        start = rospy.get_time()
        array_msg = Int32MultiArray()
        try:
            
            cv_rgb_image = self.bridge.imgmsg_to_cv2(self.rbg_data, self.rbg_data.encoding)
    
            results = self.detect_img(cv_rgb_image)
            
            #rospy.loginfo(f'Time to detection: {rospy.get_time()-start} seconds')

            if len(results.boxes.conf) > 0:
                       
                bounding_boxes = results.boxes.xyxy
                result_img = results.plot()

                x_min, y_min, x_max, y_max, x_center, y_center = self.get_center_of_box(bounding_boxes)
                self.pub_bb_image.publish(self.bridge.cv2_to_imgmsg(result_img, encoding='rgb8'))  ###Bei Jetson Verion herausnehmen (nur zu visualisierungszwecken)

                array_msg.data = [x_min, y_min, x_max, y_max, x_center, y_center,self.rbg_data.height, self.rbg_data.width]  # Example array values 
    
                
                return array_msg, True
            else:
                
                
                self.pub_bb_image.publish(self.bridge.cv2_to_imgmsg(results.orig_img, encoding='rgb8'))
                return array_msg, False
            
            
            
            


        except CvBridgeError as e:
            print(e)
            return array_msg,False
    def main(self):
        if self.detection_type == 'service':
            rospy.spin()
        else:
            array_msg = Int32MultiArray()
            while not rospy.is_shutdown():
                array_msg, success = listener.imageService(None)
                self.bb_pub.publish(array_msg)
                self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node("image_bounding_boxes")
    listener = ImageListener()
    listener.main()
    

