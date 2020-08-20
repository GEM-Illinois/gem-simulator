import pcl
import pickle
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d
fig = plt.figure(dpi=100)
ax = fig.add_subplot(111, projection='3d')

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs import point_cloud2
from std_msgs.msg import Float32

import pcl
import copy
import random
import math
import time
import imutils
from imutils import paths
from imutils.object_detection import non_max_suppression

import cv2
from cv_bridge import CvBridge

class collectData():
    def __init__(self):

        self.pointCloudSub = rospy.Subscriber("/velodyne_points", PointCloud2, self.__pointCloudHandler, queue_size=10)
        self.rawImgSub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.__rawImgHandler, queue_size=1)
        self.detectImgPub = rospy.Publisher("/pedestrian/annotate", Image, queue_size=1)

        self.cvBridge = CvBridge()
        self.__processedImg  = np.zeros((200,200,3), np.uint8)
        self.__ped = False
        self.__bbox = []
        self.pointCloudList = []



    def __pointCloudHandler(self, data):
        self.pointCloudList.append(data)




    def getProcessedImage(self):
        #cv2.imwrite('cameraImg.jpg', img)

        return self.__processedImg.copy(), self.__ped, self.__bbox

    def pedDetector(self, image):
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        image = imutils.resize(image, width=min(200, image.shape[1]), height=min(200, image.shape[0]))
        orig = image.copy()
    	# detect people in the image
    	(rects, weights) = hog.detectMultiScale(image, winStride=(1, 1),
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

        # print(pick)
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
        # cv2.imwrite('boundedIMG.jpg', self.__processedImg)

        processedROSImg = self.cvBridge.cv2_to_imgmsg(processedImg, 'bgr8')
        self.__processedImg = orig.copy()
        self.detectImgPub.publish(processedROSImg)



def main():
    rospy.init_node('dataCollection')
    rospy.sleep(3)

    collectData1 = collectData()
    x = 0
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
        img, ped, bboxes = collectData1.getProcessedImage()
        if x == 0 and ped and len(collectData1.pointCloudList) > 0:
            with open('pointCloud.data', 'wb') as filehandle:
                # store the data as binary data stream
                pickle.dump(collectData1.pointCloudList, filehandle)


            cv2.imwrite('cameraImg.jpg', img)

            with open('bboxes.data', 'wb') as filehandle:
                # store the data as binary data stream
                pickle.dump(bboxes, filehandle)
            x = 1
            print("dumped")


if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down dataCollection Node")
