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
        self.cluster_epsilon = rospy.get_param("/points_clusterer/cluster_epsilon")
        self.cluster_min_size = rospy.get_param("/points_clusterer/cluster_min_size")


        # Parameters
        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size)

        # Publishers

        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2 ** 24,
                         tcp_nodelay=True)

    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        print('points shape: ', points.shape)
        rospy.loginfo('points shape: ', points.shape)

        try:
            assert points.shape[0] == self.clusterer.labels_.first[0]
            self.clusterer.fit_predict(points)
            print('labels: ', self.clusterer.labels_)
            rospy.loginfo('labels: ', self.clusterer.labels_)
        except AssertionError as error:
            print(error)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_cluster', log_level=rospy.DEBUG)
    node = PointsCluster()
    node.run()
