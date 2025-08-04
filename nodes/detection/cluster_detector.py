#!/usr/bin/env python3

import rospy
import numpy as np

from shapely import MultiPoint
from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify, msgify

from sensor_msgs.msg import PointCloud2
from autoware_mini.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point32


BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)

class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param('~min_cluster_size')
        self.output_frame = rospy.get_param('/detection/output_frame')
        self.transform_timeout = rospy.get_param('~transform_timeout')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.objects_pub = rospy.Publisher('detected_objects', DetectedObjectArray, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('points_clustered', PointCloud2, self.cluster_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())


    def cluster_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)

        ones = np.ones((points.shape[0], 1), dtype=np.float32)
        points = np.hstack((points, ones))  # shape becomes (N, 4)
        labels = data['label']

        if msg.header.frame_id != self.output_frame:

            # fetch transform for target frame
            try:
                transform = self.tf_buffer.lookup_transform(self.output_frame, msg.header.frame_id, msg.header.stamp,
                                                            rospy.Duration(self.transform_timeout))
            except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return

        tf_matrix = numpify(transform.transform).astype(np.float32)
        # make copy of points
        points = points.copy()
        # turn into homogeneous coordinates
        points[3, :] = 1
        # transform points to target frame
        points = points.dot(tf_matrix.T)


        unique_labels = np.unique(labels)

        clusters = DetectedObjectArray()
        clusters.header.stamp = msg.header.stamp
        clusters.header.frame_id = self.output_frame

        for i in unique_labels:
            if i == -1:
                continue  # skip noise

            # create mask
            mask = (labels == i)
            # select points for one object from an array using a mask
            # rows are selected using a binary mask, and only the first 3 columns are selected: x, y, and z coordinates
            points3d = points[mask, :3]

            if len(points3d) < self.min_cluster_size:
                continue # skip sparse clusters

            centroid = np.mean(points3d, axis=0)

            # create convex hull
            points_2d = MultiPoint(points[mask, :2])
            hull = points_2d.convex_hull
            convex_hull_points = [a for hull in [[x, y, centroid[2]] for x, y in hull.exterior.coords] for a in hull]

            object = DetectedObject()

            object.centroid.x = centroid[0]
            object.centroid.y = centroid[1]
            object.centroid.z = centroid[2]

            object.convex_hull = convex_hull_points

            object.label = "unknown"
            object.id = i
            object.color = BLUE80P
            object.valid = True
            object.position_reliable = True
            object.velocity_reliable = False
            object.acceleration_reliable = False

            clusters.objects.append(object)


        self.objects_pub.publish(clusters)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cluster_detector', log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()