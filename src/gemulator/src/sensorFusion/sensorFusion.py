import copy
import time
import random
import numpy as np

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

from videoProcessing import videoProcessing
from lidarProcessing import lidarProcessing

class sensorFusion():
    def __init__(self):

        self.cvBridge = CvBridge()

        self.video = videoProcessing()
        self.lidar = lidarProcessing()

        self.sensorFusionImgPub = rospy.Publisher("/sensorFusionImg", Image, queue_size=1)
        self.__pedDists = []

    def getPedDistances(self):
        pedDists = self.__pedDists
        self.__pedDists = []
        return pedDists

    def projectTo2D(self,cameraParams, lidarPt, alpha, beta, gamma ):

            fx = cameraParams[0]
            fy = cameraParams[1]
            cx = 0#cameraParams[2]
            cy = 0#cameraParams[3]

            x = lidarPt[0]
            y = lidarPt[1]
            z = lidarPt[2]

            rx = np.array([[1,0,0],[0,np.cos(alpha), -np.sin(alpha)],[0,np.sin(alpha), np.cos(alpha)]])
            ry = np.array([[np.cos(beta),0,np.sin(beta)],[0,1,0],[-np.sin(beta),0,np.cos(beta)]])
            rz = np.array([[np.cos(gamma), -np.sin(gamma), 0],[np.sin(gamma), np.cos(gamma), 0],[0,0,1]])
            r = np.dot(rx, np.dot(ry,rz))


            projectionMatrix = np.dot(np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0]]))

            cameraCoordinates = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
            pixelCoordinates = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0]])
            lidarCameraProjectionMatrix = np.hstack((np.vstack([r,[0,0,0]]), np.array([[0],[0],[0],[1]])))
            lidarCoordinates = np.array([[y],[z],[x],[1]])


            imageCoordinates = np.dot(cameraCoordinates,np.dot(pixelCoordinates,np.dot(lidarCameraProjectionMatrix,lidarCoordinates)))

            return imageCoordinates


    def fuseSensors(self):
        cameraParams = [.4767030836014194, .4767030836014194, .4005, .4005] #(fx,fy,cx,cy)
        img, ped, bboxes = self.video.getProcessedImage()
        clusters = self.lidar.getClusteredPointCloud()

        #print(ped, bboxes, img.shape)
        if ped == False:
            #print("No ped")
            return
        # else:
        #     print(ped)

        start = time.time()

        clust_colors = []
        for clust in clusters:
            x1, y1, z1 = [],[],[]
            c = (random.random(), random.random(), random.random())
            clust_colors.append(c)

        pedImg = img.copy()
        pedImg2 = img.copy()

        clusterImages = []
        pedDists = []
        largest_clust = dict() #for each box
        max_clust_size = dict() # for each box

        bboxes = bboxes.tolist()
        # generate bounding box
        for i, rect in enumerate(bboxes):
            c = (int(clust_colors[i][0]*255), int(clust_colors[i][1]*255), int(clust_colors[i][2]*255))
            # top left, bottom right
            cv2.rectangle(pedImg2, (rect[0],rect[1]), (rect[2],rect[3]), c, 3)

        for i, clust in enumerate(clusters):
            c = (int(clust_colors[i][0]*255), int(clust_colors[i][1]*255), int(clust_colors[i][2]*255))
            clust_cnt = dict()
            imageCoordinatesList = []
            avgYPos = 0
            pedbox = False
            for pt in clust:
                imageCoordinates = self.projectTo2D(cameraParams, pt, 0, 0, 0)
                imageCoordinatesList.append(imageCoordinates)

                x = 90 - ((imageCoordinates[0]) * 100)
                y = 90 - ((imageCoordinates[1]) * 100)
                for j, rect in enumerate(bboxes):
                    if (rect[0] <= x <= rect[2]) and (rect[1] <= y <= rect[3]):
                        pedbox = True
                        avgYPos += pt[1]
                        try:
                            clust_cnt[j] += 1 # cluster points within each box
                        except KeyError:
                            clust_cnt[j] = 1

                cv2.circle(pedImg2, (x,y),1, c, -1)

            if pedbox:
                avgYPos = avgYPos/len(clust)
                pedDists.append(avgYPos)

            clusterImages.append(imageCoordinatesList)

            for key in clust_cnt.keys():
                try:
                    if clust_cnt[key] > max_clust_size[key]:
                        largest_clust[key] = i # current cluster for bbox
                        max_clust_size[key] = clust_cnt[key]
                except KeyError:
                    largest_clust[key] = i
                    max_clust_size[key] = clust_cnt[key]

        for n in largest_clust:
            #print(n)
            c = (int(clust_colors[n][0]*255), int(clust_colors[n][1]*255), int(clust_colors[n][2]*255))
            for pts in clusterImages[n]:
                # Check points in cluster
                x = 90 - ((pts[0]) * 100)
                y = 90 - ((pts[1]) * 100)
                #cv2.circle(pedImg, (x,y), 1, (0,255,0), -1)
                cv2.circle(pedImg, (x,y),1, c, -1)

        #print("fusion: {} ms".format(round(1000*(time.time() - start), 3)))
        self.__pedDists = pedDists
        projectedLidarImg = self.cvBridge.cv2_to_imgmsg(pedImg2, 'bgr8')
        self.sensorFusionImgPub.publish(projectedLidarImg)

        return






def main():
    rospy.init_node("sensorFusion")
    rospy.sleep(3)


    fusion = sensorFusion()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        fusion.fuseSensors()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down sensorFusion Node")
