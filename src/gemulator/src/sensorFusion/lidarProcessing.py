import pcl
import copy
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as axes3d
fig = plt.figure(dpi=100)
ax = fig.add_subplot(111, projection='3d')

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2



class lidarProcessing():
    def __init__(self):

        self.pointCloudSub = rospy.Subscriber("/velodyne_points", PointCloud2, self.__pointCloudHandler, queue_size=10)
        self.__clusteredPointCloud = []


    def getClusteredPointCloud(self):
        return self.__clusteredPointCloud

    def __pointCloudHandler(self, data):

        gen = point_cloud2.read_points(cloud=data, field_names=('x', 'y', 'z', 'ring'))

        lidarPt = []
        for p in gen:
            if p[0] >= 0 and p[2] > -1.9:
                lidarPt.append((p[0],p[1],p[2]))

        start = time.time()
        clusters = self.__euclidean_cluster_pcl(lidarPt)
        end = time.time()
        #print(clusters)
        self.__clusteredPointCloud = clusters

    def __euclidean_cluster_pcl(self,data):
        """
        Generates euclidean cluster and returns set of points clustered together
        """
        # cloud = pcl.load
        start = time.time()
        cloud = pcl.PointCloud()
        parr = np.array(data, dtype=np.float32)
        # np delete 3rd col on column axis
        cloud.from_array(parr)

        #print(cloud)
        # Downsample dataset with leafsize = 0.5
        # Create voxelgrid filter
        vg = cloud.make_voxel_grid_filter()
        vg.set_leaf_size(0.03, 0.03, 0.03)
        pcl_filtered = vg.filter()

        # Ground plane removal to filter ground planar pts with RANSAC method
        seg = cloud.make_segmenter()
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_MaxIterations(100)
        seg.set_distance_threshold(100)

        # Get nr points
        i = 0
        nr_points = pcl_filtered.size

        # Construct kd-tree to cluster points by k-nearest
        tree = pcl_filtered.make_kdtree()
        euclid = pcl_filtered.make_EuclideanClusterExtraction()
        euclid.set_ClusterTolerance(0.3)
        euclid.set_MinClusterSize(10)
        euclid.set_MaxClusterSize(50)
        euclid.set_SearchMethod(tree)
        clust_indices = euclid.Extract()

        clusters = list()
        points = list()
        # Construct pcl clusters
        pcl_cluster = pcl.PointCloud()
        for j, indices in enumerate(clust_indices):
            points = np.zeros((len(indices), 3),
                                dtype=np.float32)
            for i, idx in enumerate(indices):
                #x , y, z
                points[i][0] = pcl_filtered[idx][0]
                points[i][1] = pcl_filtered[idx][1]
                points[i][2] = pcl_filtered[idx][2]

            clusters.append(points)
            # pcl_cluster.from_array(points)
            # ss = "./data/clust_" + str(j) + ".pcd"
            # pcl.save(pcl_cluster, ss)

        #print("clustering: {} ms".format(round(1000*(time.time() - start), 3)))
        return clusters




def main():
    rospy.init_node('lidarProcessing')
    rospy.sleep(3)

    LiDAR = lidarProcessing()


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        #print(LiDAR.getClusteredPointCloud())
        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down lidarProcessing Node")
