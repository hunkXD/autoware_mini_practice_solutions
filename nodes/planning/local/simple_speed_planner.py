#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
import threading
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, Log
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from autoware_mini.geometry import project_vector_to_heading, get_distance_between_two_points_2d
from scipy.interpolate import interp1d
from shapely.geometry import LineString, Point

class SpeedPlanner:

    def __init__(self):

        # parameters
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")
        self.distance_to_car_front = rospy.get_param("distance_to_car_front")

        # variables
        self.collision_points = None
        self.current_position = None
        self.current_speed = None

        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)

        ts.registerCallback(self.collision_points_and_path_callback)

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            with self.lock:
                collision_points = numpify(collision_points_msg) if len(collision_points_msg.data) > 0 else np.array([])
                current_position = self.current_position
                current_speed = self.current_speed

            if current_position is None or current_speed is None:
                return

            if collision_points.size == 0:
                self.local_path_pub.publish(local_path_msg)
                return

            local_path_xyz = np.array([[wp.position.x, wp.position.y] for wp in local_path_msg.waypoints])
            path_linestring = LineString(local_path_xyz)

            collision_points_geom = [Point(p['x'], p['y']) for p in collision_points]

            distances_to_collisions = [path_linestring.project(p) for p in collision_points_geom]

            # 1. Compute heading angle at each collision point's distance on the path
            collision_point_path_headings = [
                self.get_heading_at_distance(path_linestring, distance)
                for distance in distances_to_collisions
            ]

            # 2. Project object velocity onto ego's heading
            collision_point_velocities = []
            for heading, p in zip(collision_point_path_headings, collision_points):
                velocity_vector = Vector3(p['vx'], p['vy'], p['vz'])

                # Project object velocity onto path heading
                projected_velocity = self.project_vector_to_heading(heading, velocity_vector)

                # Actual object speed (norm of vector)
                object_speed = np.linalg.norm([p['vx'], p['vy']])

                rospy.loginfo_throttle(5.0, f"Object speed: {object_speed:.2f} m/s, "
                                            f"Projected speed along path: {projected_velocity:.2f} m/s")

                collision_point_velocities.append(projected_velocity)

            # Convert to NumPy array for further use if needed
            collision_point_velocities = np.array(collision_point_velocities)

            for d, p in zip(distances_to_collisions, collision_points):
                target_velocities = [
                    max(0.0,
                        math.sqrt(current_speed ** 2 + 2 * self.default_deceleration * (
                                    d - self.distance_to_car_front - p['distance_to_stop']))
                        )
                ]
                min_target_velocity = min(target_velocities)

                # Check if any point requires a hard stop (deceleration_limit == 0 or close to 0)
                if d <= p['distance_to_stop']:
                    path = Path()
                    path.header = local_path_msg.header
                    path.waypoints = []
                    path.closest_object_velocity = 0.0
                    path.is_blocked = True
                    self.local_path_pub.publish(path)
                    return

            for i, wp in enumerate(local_path_msg.waypoints):
                wp.speed = min(min_target_velocity, wp.speed)


            # Update the lane message with the calculated values
            path = Path()
            path.header = local_path_msg.header
            path.waypoints = local_path_msg.waypoints
#            path.closest_object_distance = closest_object_distance  # Distance to the collision point with lowest target velocity (also closest object for now)
            path.closest_object_velocity = 0  # Velocity of the collision point with lowest target velocity (0)
            path.is_blocked = True
#            path.stopping_point_distance = closest_object_distance  # Stopping point distance can be set to the distance to the closest object for now
#            path.collision_point_category = collision_point_category  # Category of collision point with lowest target velocity
            self.local_path_pub.publish(path)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def get_heading_at_distance(self, linestring, distance):
        """
        Get heading of the path at a given distance
        :param distance: distance along the path
        :param linestring: shapely linestring
        :return: heading angle in radians
        """

        point_after_object = linestring.interpolate(distance + 0.1)
        # if distance is negative it is measured from the end of the linestring in reverse direction
        point_before_object = linestring.interpolate(max(0, distance - 0.1))

        # get heading between two points
        return math.atan2(point_after_object.y - point_before_object.y, point_after_object.x - point_before_object.x)


    def project_vector_to_heading(self, heading_angle, vector):
        """
        Project vector to heading
        :param heading_angle: heading angle in radians
        :param vector: vector
        :return: projected vector
        """

        return vector.x * math.cos(heading_angle) + vector.y * math.sin(heading_angle)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speed_planner')
    node = SpeedPlanner()
    node.run()