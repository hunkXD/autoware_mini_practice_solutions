#!/usr/bin/env python3

import rospy
import shapely
import math
import numpy as np
import threading
from ros_numpy import msgify, numpify
from autoware_mini.msg import Path, DetectedObjectArray, TrafficLightResultArray
from sensor_msgs.msg import PointCloud2
from shapely import LineString, Polygon, BufferCapStyle
from autoware_mini.geometry import get_speed_from_velocity
from autoware_mini.lanelet2 import load_lanelet2_map, get_traffic_light_stop_lines

DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('vx', np.float32),
    ('vy', np.float32),
    ('vz', np.float32),
    ('distance_to_stop', np.float32),
    ('deceleration_limit', np.float32),
    ('category', np.int32)
])

class CollisionPointsManager:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")
        self.braking_safety_distance_stopline = rospy.get_param("~braking_safety_distance_stopline", 5.0)
        self.lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.tfl_force_stop_speed_limit = rospy.get_param("~tfl_force_stop_speed_limit", 5.0)
        self.tfl_maximum_deceleration = rospy.get_param("~tfl_maximum_deceleration", 4.0)

        # variables
        self.detected_objects = None

        self.stopline_statuses = {}
        self.lanelet2_map = load_lanelet2_map(self.lanelet2_map_path)
        self.all_stoplines = get_traffic_light_stop_lines(self.lanelet2_map)

        self.stopline_id_to_position = {}
        stoplines = get_traffic_light_stop_lines(self.lanelet2_map)

        for stopline_id, stopline in stoplines.items():
            # Take first point of line string
            self.stopline_id_to_position[stopline_id] = (stopline.coords[0][0], stopline.coords[0][1])

        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_collision_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/detection/traffic_light_status', TrafficLightResultArray, self.traffic_light_status_callback,
                         queue_size=1, tcp_nodelay=True)

    def traffic_light_status_callback(self, msg):
        stopline_statuses = {}
        for result in msg.results:
            stopline_statuses[result.stopline_id] = result.recognition_result

        self.stopline_statuses = stopline_statuses


    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def path_callback(self, msg):
        with self.lock:
            detected_objects = self.detected_objects
        collision_points = np.array([], dtype=DTYPE)

        if detected_objects is None:
            rospy.logwarn("No detected objects not received!")
            return

        # Publish empty PointCloud
        if msg.waypoints != [] or detected_objects != []:
            path_linestring = LineString([(path.position.x, path.position.y) for path in msg.waypoints])
            local_path_buffer = path_linestring.buffer(self.safety_box_width / 2, cap_style="flat")
            shapely.prepare(local_path_buffer)

            for object in detected_objects:
                object_polygon = shapely.polygons(np.array(object.convex_hull).reshape(-1, 3))

                if local_path_buffer.intersects(object_polygon):
                    intersection_result = object_polygon.intersection(local_path_buffer)
                    intersection_points = shapely.get_coordinates(intersection_result)
                    object_speed = get_speed_from_velocity(object.velocity)

                    for x, y in intersection_points:
                        collision_points = np.append(collision_points, np.array(
                            [(x, y, object.centroid.z, object.velocity.x, object.velocity.y, object.velocity.z,
                              self.braking_safety_distance_obstacle, np.inf,
                              3 if object_speed < self.stopped_speed_limit else 4)], dtype=DTYPE))

            collision_points_msg = msgify(PointCloud2, collision_points)

            # === Traffic Light Stopline Collision Points ===
            if hasattr(self, "stopline_statuses") and hasattr(self, "stopline_id_to_position"):
                for stopline_id, (x, y) in self.stopline_id_to_position.items():
                    status = self.stopline_statuses.get(stopline_id, -1)

                    # Only consider RED lights
                    if status not in [0]:
                        continue

                    # Check if stopline is near the path
                    stopline_point = shapely.Point(x, y)
                    if not local_path_buffer.contains(stopline_point):
                        continue

                    speed_mps = getattr(self, "current_speed", 0.0)
                    decel_limit = (
                        np.inf
                        if speed_mps < (self.tfl_force_stop_speed_limit / 3.6)
                        else self.tfl_maximum_deceleration
                    )

                    # Append traffic light stopline collision point
                    collision_points = np.append(collision_points, np.array(
                        [(x, y, 0.0, 0.0, 0.0, 0.0,
                          self.braking_safety_distance_stopline, decel_limit,
                          2)],  # Category 2 = Traffic Light Stopline
                        dtype=DTYPE))


            collision_points_msg.header = msg.header
            # Publish the merged collision points
            self.local_path_collision_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('collision_points_manager')
    node = CollisionPointsManager()
    node.run()