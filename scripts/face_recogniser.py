from __future__ import print_function
import roslib
import face_recognition
import pickle
import time
import cv2
import rospy
import sys
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FaceRecogniser(object):
   def __init__(self):
      
      print("[INFO] loading encodings...")
      self.data = pickle.loads(open("encodings.pickle", "rb").read())
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
      self.tracker = cv2.TrackerMedianFlow_create()
      self.newFacialDetection = True  # specifies if tracker has to be reinitialised with new bounding box
      self.pub = rospy.Publisher('angle', Float32, queue_size=1)
      #faceBox = []  # initial location of face to be tracked

   def callback(self,msg):
      try:
         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
      except CvBridgeError as e:
         print(e)

      self.track(cv_image)

   def track(self, _frame, window_name="Tracking"):
      

      if self.newFacialDetection:
         faceBox = self.detect(_frame)
         print(faceBox)
         _ok = self.tracker.init(_frame, faceBox)

         if not _ok:
            print ("Could not initialise tracker")
            return
            #sys.exit()

         self.newFacialDetection = False
         return

      _ok, _box = self.tracker.update(_frame)

      if _ok:
         p1 = (int(_box[0]), int(_box[1]))  # top left corner
         p2 = (int(_box[0] + _box[2]), int(_box[1] + _box[3]))  # bottom right corner
         cv2.rectangle(_frame, p1, p2, (255, 0, 0), 2, 1)  # draw a rectangle on the frame
         angle = self.get_angle(_frame, _box)
         self.pub.publish(angle)
         print (angle)
      else:
         cv2.putText(_frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
         self.tracker = cv2.TrackerMedianFlow_create()
         self.newFacialDetection = True
         

      cv2.imshow(window_name, _frame)
      cv2.waitKey(3)

   def detect(self, img):
      
      rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
      boxes = face_recognition.face_locations(rgb, model="hog")
      encodings = face_recognition.face_encodings(rgb, boxes)
      names = []

      for encoding in encodings:
         matches = face_recognition.compare_faces(self.data["encodings"], encoding)
         name = "Unknown"
         
         if True in matches:
            matchedIdxs = [i for (i, b) in enumerate(matches) if b]
            counts = {}
            for i in matchedIdxs:
               name = self.data["names"][i]
               counts[name] = counts.get(name, 0) + 1
            name = max(counts, key=counts.get)

         names.append(name)
      x = 0
      y = 0
      w = 0
      h = 0
      
      for ((top, right, bottom, left), name) in zip(boxes, names):
         x = left
         y = top
         w = right - left
         h = bottom - top
         rectBox = cv2.rectangle(img, (left, top), (right, bottom),(0, 255, 0), 2)
         y = top - 15 if top - 15 > 15 else top + 15
         cv2.putText(img, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        
      return (int(x), int(y), int(w), int(h))
   
   def get_angle(self,_frame, _box): 

      box_centre = _box[0] + _box[2] / 2, _box[1] + _box[3] / 2
      window_width = _frame.shape[1]

      return (box_centre[0] - (window_width / 2)) / (window_width / 2) * (70/ 2)   

   def run(self):
      rospy.spin()

if __name__ == "__main__":
   fr = FaceRecogniser()
   rospy.init_node('face_recogniser', anonymous=True)
   fr.run()
   


