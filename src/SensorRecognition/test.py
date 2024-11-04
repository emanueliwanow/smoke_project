from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

start_loading_model = time.time() 
model = YOLO("best.pt")  #mit car detection ersetzen
print("Model nach "+ str(time.time() - start_loading_model) + " sekunden geladen")

img = cv2.imread("image.jpeg")
bridge = CvBridge()
#cv_rgb_image = bridge.imgmsg_to_cv2(img, img.encoding)
start_loading_model = time.time() 
result = model(img, verbose=False, conf = 0.3)
#print(result)
print("Got result after "+ str(time.time() - start_loading_model) + " seconds")
