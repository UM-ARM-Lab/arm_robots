"""!
Class to represent the camera.
"""

import cv2
import time
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix
import block_colors 
from math import atan2, fabs
import Queue


class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self):
        """!
        @brief      Constructs a new instance.
        """
        self.VideoFrame = np.zeros((480,640,3)).astype(np.uint8)
        self.VideoFrameHSV = np.zeros((480,640,3), dtype=np.uint8)
        self.TagImageFrame = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((480,640)).astype(np.uint16)
        self.DepthFramelpf = np.zeros((480,640)).astype(np.uint16)
        self.DepthFrameQueue = []
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthFrameRGB=np.array([])

        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.intrinsicCalibrated = False
        self.extrinsicCalibrated = False
        self.cameraCalibrated = False
        self.last_click = np.array([0.0,0.0,0.0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.tag_detections = np.array([])
        self.extrinsic = np.array([[0, 1, 0, 0],
                                  [-1, 0, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])

        """ block info """
        self.block_contours = np.array([])
        self.block_detections = {}

    def uvd2workspace(self, pi = None):
      if pi is None:
        pi = np.copy(self.last_click)
      #camera frame coordinate
      pc = np.zeros(3)
      pc[0] = (pi[0] - self.intrinsic[0,2])/self.intrinsic[0,0]*pi[2]
      pc[1] = (pi[1] - self.intrinsic[1,2])/self.intrinsic[1,1]*pi[2]
      pc[2] = pi[2]
      #print "pi", pi
      #pc = np.linalg.inv(self.intrinsic).dot(np.array([pi[0], pi[1], 1]).T)*pi[2]
      pr = np.linalg.inv(self.extrinsic).dot(np.append(pc,1))

      #round to nearest multiples of 3.9cm
      h = 0.038
      rounded_depth = 0
      while pr[2] > h/2:
        pr[2] -= h
        rounded_depth += h
      while pr[2] < -h/2:
        pr[2] += h
        rounded_depth -= h
      pr[2] = rounded_depth
      return pr[0:3]

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame,self.block_contours,-1,(255,0,255),3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[...,0] = self.DepthFrameRaw
        self.DepthFrameHSV[...,1] = 0x9F
        self.DepthFrameHSV[...,2] = 0xFF
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png",cv2.IMREAD_UNCHANGED),cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (640, 480))
            img = QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertQtDepthFrame(self):
       """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
       try:
           img = QImage(self.DepthFrameRGB,
                            self.DepthFrameRGB.shape[1],
                            self.DepthFrameRGB.shape[0],
                            QImage.Format_RGB888
                            )
           return img
       except:
           return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (640, 480))
            img = QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

                    TODO: Rewrite this function to take in an arbitrary number of coordinates and find the transform without
                    using cv2 functions

        @param      coord1  The coordinate 1
        @param      coord2  The coordinate 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)


    def registerDepthFrame(self, frame):
        """!
        @brief      Transform the depth frame to match the RGB frame

                    TODO: Using an Affine transformation, transform the depth frame to match the RGB frame using
                    cv2.warpAffine()

        @param      frame  The frame

        @return     { description_of_the_return_value }
        """
        return frame

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

        @param      file  The file
        """
        pass

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        # Reset detections
        self.block_detections = {}
        self.block_contours = np.array([])

        # Convert to HSV
        self.VideoFrameHSV = cv2.cvtColor(self.VideoFrame, cv2.COLOR_RGB2HSV)
        blurred = cv2.GaussianBlur(self.VideoFrameHSV, (5,5), 0)
        #depth_mask = self.detectBlocksInDepthImage()

        for color, bounds in block_colors.COLORS.iteritems():
            # Construct mask
            mask = np.zeros_like(blurred[..., 0], dtype=np.uint8)
            for lb, ub in bounds:
                color_mask = cv2.inRange(blurred, lb, ub)
                mask = cv2.bitwise_or(mask, color_mask)

            # Add depth mask
            # mask = cv2.bitwise_and(color_mask, depth_mask)

            # Clean with morphological filters
            # TODO This may not be as necessary with the depth mask used
            kernel = np.ones((3,3), np.uint8)
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel, iterations=2)

            # Find contours 
            _, contours,  _ = cv2.findContours(cleaned, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE) # Ignoring returned hierarchy  


            # Filter contours by size and aspect ratio
            for cnt in contours:
                # Find minimum area rectangle
                rect = cv2.minAreaRect(cnt)
                pos, size, orientation = rect

                # Verify that contour is roughly square and of reasonable size
                h,w = size
                aspect_ratio = h / float(w)
                area = h * w

                u, v = map(int, pos)

                # Aspect ratio for a square block viewed from most perspectives shoud be between sqrt(2)/2 and sqrt(2)
                if 0.71 < aspect_ratio < 1.41 and area >= 400:
                    #contour_mask = np.zeros(cleaned.shape, dtype=np.uint8)
                    #cv2.drawContours(contour_mask, [cnt], -1, (255), 2)
                    #cv2.floodFill(contour_mask, None, (u, v), (255))

                    #signed_hsv = np.array(blurred, dtype=np.int16)
                    #mean, stdev = cv2.meanStdDev(signed_hsv, mask=contour_mask)

                    #centroid_color = self.VideoFrameHSV[v, u]

                    # Find the color that best matches the contour's mean color
                    #color = min(block_colors.COLOR_MEANS.iteritems(), 
                    #   key=lambda (test_color, color_vals): np.linalg.norm(color_vals - centroid_color[:2]))[0]

                    orientation = (orientation % 90) * np.pi / 180
                    orientation += self.angle_offset

                    # Get XYZ coordinates
                    d = self.DepthFrameRaw[v, u] / 1000 
                    xyz = self.uvd2workspace(np.array([u, v, d]))

                    # If the 'block' is too low, skip it
                    if xyz[2] < 0.01:
                      continue
                    # If the block is off of the board, skip it
                    if abs(xyz[1]) > 0.305 or xyz[0] < -0.235 or xyz[0] > 0.375:
                      continue
                    # Else if the block is black and close to 0,0, skip it (it is probably robot base)
                    if (color == "black" or color == "blue") and xyz[0] < 0.075 and abs(xyz[1]) < 0.075:
                      continue

                    # Store in self.block_detections
                    xyz_phi = np.append(xyz, orientation)
                    blocks = self.block_detections.get(color, [])
                    blocks.append(xyz_phi)
                    self.block_detections[color] = blocks

                    np.append(self.block_contours, cnt)



    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        # Blur and equalize histogram to remove noise and improve contrast
        blurred = cv2.GaussianBlur(self.DepthFramelpf, (5,5), 0)

        #convert to 8 bit
        smaller = cv2.normalize(blurred, cv2.NORM_MINMAX, 0, 255).astype(np.uint8)
        
        equalized = cv2.equalizeHist(smaller)

        # Threshold to remove arm
        _, dist = cv2.threshold(equalized, 20, 255, cv2.THRESH_BINARY)
        masked = cv2.bitwise_or(equalized, equalized, mask=dist)

        # Invert for double-sided threshold
        masked = 1 - masked

        # Threshold to remove floor
        _, dist = cv2.threshold(masked, 255 - 42, 255, cv2.THRESH_BINARY)

        # Clean with morphological filters
        kernel = np.ones((5,5), dtype=np.uint8)
        cleaned = cv2.morphologyEx(dist, cv2.MORPH_OPEN, kernel, iterations=4)
        dilated = cv2.morphologyEx(cleaned, cv2.MORPH_DILATE, kernel, iterations=2)

        # Construct mask
        return dilated 


    def single_tag_calib(self, z_bias = -0.02):
        """!
        @brief      calculate the extrinsic matrix from the transfrom camera->tag and tag->robot

        """ 
        #take average of n tca position and orientation (poses of robot apriltag in camera frame)
        n = 20 # number of samples
        #pose of apriltag in camera frame
        tca_position = np.zeros((3))
        tca_quaternion = np.zeros((4)) #(x, y, z, w)
        for i in range(n):
          data = self.tag_detections
          flag = False
          for detection in data.detections:
            if detection.id[0]==1: # id 1 is the tag on the robot
              flag = True
              tca_position[0]+=detection.pose.pose.pose.position.x
              tca_position[1]+=detection.pose.pose.pose.position.y
              tca_position[2]+=detection.pose.pose.pose.position.z
              tca_quaternion[0]+=detection.pose.pose.pose.orientation.x
              tca_quaternion[1]+=detection.pose.pose.pose.orientation.y
              tca_quaternion[2]+=detection.pose.pose.pose.orientation.z
              tca_quaternion[3]+=detection.pose.pose.pose.orientation.w
          rospy.sleep(0.1)
          if flag == False:
            print("Tag on the robot not detected, tag blocked by obstacles.")
        tca_position/=n
        tca_quaternion/=n
        #convert pose to 4x4 homogeneous form
        tca = quaternion_matrix(tca_quaternion)
        tca[:3,3] = tca_position.T
        #tra, pose of apriltag in robot frame, calculated from drawing
        tra = np.array([[ 0, 1, 0, -0.1416],
                        [-1, 0, 0, 0],
                        [ 0, 0, 1, 0.037],
                        [ 0, 0, 0, 1]])
        tcr = tca.dot(np.linalg.inv(tra))
        tcr[2,3] += z_bias
        self.extrinsic = tcr
        self.extrinsicCalibrated = True
        print("Entrinsic Parameter:\n", tcr)

    def multi_tag_calib(self, z_bias = 0.02):
            """!
            @brief      calculate the extrinsic matrix from the transfrom camera->tag and tag->robot

            """ 
            #take average of n tag positions in camera frame (tag 1-4)
            n = 20 # number of samples
            #pose of apriltag 1 in camera frame
            tca = np.zeros((4,4))
            #position of all tags in camera frame, [pca1, pca2, pca3, pca4]
            tag_pos = np.zeros((3,4))
            for i in range(n):
              data = self.tag_detections
              if not len(data.detections)== 4:
                print "Not all tags detected, maybe blocked, iteration {}".format(i) 
              for detection in data.detections:         
                  tag_pos[0,detection.id[0]-1]+=detection.pose.pose.pose.position.x
                  tag_pos[1,detection.id[0]-1]+=detection.pose.pose.pose.position.y
                  tag_pos[2,detection.id[0]-1]+=detection.pose.pose.pose.position.z
              rospy.sleep(0.1)
            tag_pos = tag_pos/n
            #tag_pos[2,1:] = np.array([0.975, 0.983, 0.962])
            #print "tag_pos", tag_pos
            #translation is pca1
            tca[:3,3] = tag_pos[:,0]
            #calculate rotation
            v1 = tag_pos[:,3] - tag_pos[:,1]
            v2 = tag_pos[:,2] - tag_pos[:,1]
            i = v1 / np.linalg.norm(v1)
            j = v2 / np.linalg.norm(v2)
            k = np.cross(i,j)
            #print i, j, k
            tca[:3,:3] = np.vstack((i,j,k)).T
            tca[3,3] = 1
            #print "tca: ", tca
            #tra, pose of apriltag in robot frame, calculated from drawing
            tra = np.array([[ 0, 1, 0, -0.1416],
                            [-1, 0, 0, 0],
                            [ 0, 0, 1, 0.037],
                            [ 0, 0, 0, 1]])
            tcr = tca.dot(np.linalg.inv(tra))
            tcr[2, 3] += z_bias
            self.extrinsic = tcr
            self.extrinsicCalibrated = True
            print "Entrinsic Parameter:\n", tcr   

    #tca_position calculated from the position of tag 1
    #tca_orientation calculated from position of tag 2-4 
    def multi_tag_depth_calib(self, z_bias = -0.004):
        """!
        @brief      calculate the extrinsic matrix from the transfrom camera->tag and tag->robot

        """ 
        #take average of n tag positions in camera frame (tag 1-4)
        n = 20 # number of samples
        #pose of apriltag 1 in camera frame
        tca = np.zeros((4,4))
        #position of all tags in camera frame, [pca1, pca2, pca3, pca4]
        tag_pos = np.zeros((3,4))
        for i in range(n):
          data = self.tag_detections
          if not len(data.detections)== 4:
            print "Not all tags detected, maybe blocked, iteration {}".format(i) 
          for detection in data.detections:         
              tag_pos[0,detection.id[0]-1]+=detection.pose.pose.pose.position.x
              tag_pos[1,detection.id[0]-1]+=detection.pose.pose.pose.position.y
              tag_pos[2,detection.id[0]-1]+=detection.pose.pose.pose.position.z
          rospy.sleep(0.1)
        tag_pos = tag_pos/n
        #tag_pos[2,1:] = np.array([0.975, 0.983, 0.962])
        #convert tag_pos to image frame
        tag_pos_i = self.intrinsic.dot(tag_pos)/tag_pos[2,:]
        #print "tag_pos_i", tag_pos_i
        #print "tag_pos", tag_pos
        #print "tag_pos", tag_pos
        #find the depth of each tag [u, v] (in tag_pos_i)
        for i in range(4):
          tag_pos[2,i] = self.get_depth(int(tag_pos_i[0,i]), int(tag_pos_i[1,i]))
        #print "tag_pos2:", tag_pos
        tca[:3,3] = tag_pos[:,0]
        #calculate rotation
        v1 = tag_pos[:,3] - tag_pos[:,1]
        v2 = tag_pos[:,2] - tag_pos[:,1]
        i = v1 / np.linalg.norm(v1)
        j = v2 / np.linalg.norm(v2)
        k = np.cross(i,j)
        #print i, j, k
        tca[:3,:3] = np.vstack((i,j,k)).T
        tca[3,3] = 1
        #print "tca: ", tca
        #tra, pose of apriltag in robot frame, calculated from drawing
        tra = np.array([[ 0, 1, 0, -0.1416],
                        [-1, 0, 0, 0],
                        [ 0, 0, 1, 0.037],
                        [ 0, 0, 0, 1]])
        tcr = tca.dot(np.linalg.inv(tra))
        tcr[2, 3] += z_bias
        self.extrinsic = tcr
        self.angle_offset = atan2(self.extrinsic[1,0], self.extrinsic[0,0])+np.pi*1/2
        print "angle_offset: ", self.angle_offset*180/np.pi
        self.extrinsicCalibrated = True
        self.cameraCalibrated = True
        print "Entrinsic Parameter:\n", tcr        

    def get_depth(self, x, y, n=5, gap = 0.04):
      d = 0
      for i in range(n):
        d += self.DepthFrameRaw[y, x]/1000.0 #in meter
        rospy.sleep(gap)
      return d/n

class ImageListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback)
    self.camera = camera

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.VideoFrame = cv_image

class TagImageListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback)
    self.camera = camera

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.TagImageFrame = cv_image

class TagDetectionListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.tag_sub = rospy.Subscriber(topic,AprilTagDetectionArray,self.callback)
    self.camera = camera
  def callback(self,data):
    self.camera.tag_detections = data
    #for detection in data.detections:
       #print(detection.id[0])
       #print(detection.pose.pose.pose.position)

class IntrinsicListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.tag_sub = rospy.Subscriber(topic, CameraInfo, self.callback)
    self.camera = camera
    self.camera.intrinsic_set_flag = False

  def callback(self, data):
    if self.camera.intrinsicCalibrated == False:
      self.camera.intrinsic = np.reshape(np.asarray(data.K),(3,3))
      print "Intrinsic Parameter:\n", self.camera.intrinsic
      self.camera.intrinsicCalibrated= True

class DepthListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback_lpf_avg)
    self.camera = camera

  def callback(self,data):
    try:
      cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.DepthFrameRaw += cv_depth
    self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
    self.camera.ColorizeDepthFrame()

  def callback_lpf(self,data, alpha = 0.7):
    try:
      cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.DepthFrameRaw = alpha * self.camera.DepthFrameRaw + (1 - alpha) * cv_depth
    self.camera.ColorizeDepthFrame()

  #moving average of lpf depth images
  def callback_lpf_avg(self,data, alpha = 0.7, window = 5):
    try:
      cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.DepthFramelpf = alpha * self.camera.DepthFramelpf + (1 - alpha) * cv_depth
    self.camera.DepthFrameQueue.insert(0, self.camera.DepthFramelpf)
    if len(self.camera.DepthFrameQueue) > window:
        self.camera.DepthFrameQueue.pop()
    self.camera.DepthFrameRaw = sum(self.camera.DepthFrameQueue)/len(self.camera.DepthFrameQueue)
    self.camera.ColorizeDepthFrame()

class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent) 
        self.camera = camera
        self.label = ""
        self.label2 = ""
        self.new_click = False
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        intrinsic_topic = "/camera/color/camera_info"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic, self.camera)
        intrinsic_listener = IntrinsicListener(intrinsic_topic, camera)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        while True:
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            if((rgb_frame != None)&(depth_frame != None)):
                self.updateFrame.emit(rgb_frame, depth_frame, tag_frame)
            time.sleep(0.03)
            if __name__ == '__main__':
                # if self.camera.extrinsicCalibrated:
                #     self.camera.blockDetector()
                #     self.camera.processVideoFrame()
                img = cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR)
                self.put_text(img)
                cv2.setMouseCallback('Image window', self.click_callback)
                cv2.imshow("Image window", img)
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow("Tag window", cv2.cvtColor(self.camera.TagImageFrame,cv2.COLOR_RGB2BGR))           
                cv2.waitKey(3)
                time.sleep(0.03)
  
    #show mouse pixel value [u,v,d] as well as workspace coordinate on the image window, 
    def put_text(self, img):
      if self.new_click == True:
        x = self.camera.last_click[0]
        y = self.camera.last_click[1]
        d = self.camera.last_click[2]
        self.label = "u: {:.0f}, v: {:.0f}, d: {:.2f} cm".format(x, y, d*100)
        pr = self.camera.uvd2workspace()
        self.label2 = "x: {:.1f}cm, y: {:.1f},cm z: {:.1f} cm".format(pr[0]*100, pr[1]*100, pr[2]*100)
        self.new_click = False
      cv2.putText(img, self.label, (50, 450), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), 2)
      cv2.putText(img, self.label2, (50, 462), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), 2)

    def click_callback(self, event, x, y, flags, params, n = 5):
      if event == cv2.EVENT_LBUTTONDBLCLK:
        self.camera.last_click[0] = x #in pixel 
        self.camera.last_click[1] = y #in pixel 
        self.camera.last_click[2] = self.camera.get_depth(x, y)
        self.camera.uvd2workspace()
        self.new_click= True

if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    #camera.single_tag_calib()
    #camera.multi_tag_calib()
    camera.multi_tag_depth_calib()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()
