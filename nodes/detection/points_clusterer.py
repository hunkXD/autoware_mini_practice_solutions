#! /usr/bin/env python3

import rospy
from docutils.languages.zh_cn import labels

from sensor_msgs.msg import PointCloud2
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from ros_numpy import numpify, msgify

from sklearn.cluster import DBSCAN

import numpy as np

class PointsCluster:
    def __init__(self):

        # Reading in the parameter values
        self.cluster_epsilon = rospy.get_param("~cluster_epsilon")
        self.cluster_min_size = rospy.get_param("~cluster_min_size")


        # Parameters
        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size)

        # Publishers
        self.cluster_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2 ** 24,
                         tcp_nodelay=True)

    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        try:
            labels = self.clusterer.fit_predict(points)
            assert points.shape[0] == len(labels)
        except AssertionError as error:
            print(error)

        idx = labels != -1

        clusters = labels[idx]
        points_clustered = points[idx]

        # concatenate points with labels
        points_labeled = np.concatenate([points_clustered, clusters[:, np.newaxis]], axis=1)


        # convert labelled points to PointCloud2 format
        data = unstructured_to_structured(points_labeled, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.int32)
        ]))

        # publish clustered points message
        cluster_msg = msgify(PointCloud2, data)
        cluster_msg.header.stamp = msg.header.stamp
        cluster_msg.header.frame_id = msg.header.frame_id

        self.cluster_pub.publish(cluster_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_cluster', log_level=rospy.INFO)
    node = PointsCluster()
    node.run()
