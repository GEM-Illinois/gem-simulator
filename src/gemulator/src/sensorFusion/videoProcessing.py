import time
import numpy as np

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

import cv2
from cv_bridge import CvBridge

import imutils
from imutils import paths
from imutils.object_detection import non_max_suppression



class videoProcessing():
    def __init__(self):

        self.cvBridge = CvBridge()
        self.rawImgSub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.__rawImgHandler, queue_size=1)
        self.detectImgPub = rospy.Publisher("/pedestrian/annotate", Image, queue_size=1)

        self.__processedImg  = np.zeros((200,200,3), np.uint8)
        self.__ped = False
        self.__bbox = []


    def getProcessedImage(self):
        return self.__processedImg, self.__ped, self.__bbox

    def pedDetector(self, image):
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        image = imutils.resize(image, width=min(200, image.shape[1]), height=min(200, image.shape[0]))
        orig = image.copy()
    	# detect people in the image
    	(rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
    		padding=(8, 8), scale=1.01)
    	# draw the original bounding boxes
    	for (x, y, w, h) in rects:
    		cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
    	# apply non-maxima suppression to the bounding boxes using a
    	# fairly large overlap threshold to try to maintain overlapping
    	# boxes that are still people
    	rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    	pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # draw the final bounding boxes
    	for (xA, yA, xB, yB) in pick:
    		cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

        #print(pick)
        if(len(pick) == 0):
            self.__ped = False
            self.__bbox = []
        else:
            self.__ped=True
            self.__bbox = pick

        return image, orig


    def __rawImgHandler(self, data):
        cvImg = self.cvBridge.imgmsg_to_cv2(data, 'bgr8')

        rawCvImg = cvImg.copy()
        start = time.time()
        processedImg, orig = self.pedDetector(rawCvImg)
        end = time.time()
        #print(end-start)
        #cv2.imwrite('boundedIMG.jpg', processedImg)

        processedROSImg = self.cvBridge.cv2_to_imgmsg(processedImg, 'bgr8')
        self.__processedImg = orig.copy()
        self.detectImgPub.publish(processedROSImg)





def main():
    rospy.init_node("videoProcessing")
    rospy.sleep(3)

    video = videoProcessing()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down videoProcessing Node")
