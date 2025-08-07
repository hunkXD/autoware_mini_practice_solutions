#!/usr/bin/env python3

import rospy
import shapely
import math
import numpy as np
import threading
from ros_numpy import msgify, numpify
from autoware_mini.msg import Path, DetectedObjectArray, TrafficLightResultArray
from sensor_msgs.msg import PointCloud2
from shapely import LineString, Polygon, BufferCapStyle, Point
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
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal", 3.0)

        # variables
        self.detected_objects = None
        self.goal_waypoint = None

        self.stopline_statuses = {}
        self.lanelet2_map = load_lanelet2_map(self.lanelet2_map_path)
        # Extract all stop lines and signals from the lanelet2 map
        all_stoplines = get_stoplines(self.lanelet2_map)
        self.trafficlights = get_stoplines_trafficlights(self.lanelet2_map)
        # If stopline_id is not in self.signals then it has no signals (traffic lights)
        self.tfl_stoplines = {k: v for k, v in all_stoplines.items() if k in self.trafficlights}

        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_collision_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('/detection/traffic_light_status', TrafficLightResultArray, self.traffic_light_status_callback,
                         queue_size=1, tcp_nodelay=True)

        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=1,
                                                tcp_nodelay=True)

    def traffic_light_status_callback(self, msg):
        stopline_statuses = {}
        for result in msg.results:
            stopline_statuses[result.stopline_id] = result.recognition_result

        self.stopline_statuses = stopline_statuses


    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def global_path_callback(self, msg):
        if msg.waypoints:
            self.goal_waypoint = msg.waypoints[-1]  # last waypoint as goal point
        else:
            self.goal_waypoint = None

    def path_callback(self, msg):
        with self.lock:
            detected_objects = self.detected_objects
        collision_points = np.array([], dtype=DTYPE)

        if detected_objects is None:
            rospy.logwarn("No detected objects not received!")

        if msg.waypoints is None:
            #publish empty waypoints
            empty_msg = msgify(PointCloud2, np.array([], dtype=DTYPE))
            empty_msg.header = msg.header
            self.local_path_collision_pub.publish(empty_msg)
            return

        path_linestring = LineString([(path.position.x, path.position.y) for path in msg.waypoints])
        local_path_buffer = path_linestring.buffer(self.safety_box_width / 2, cap_style="flat")

        if detected_objects is not None and len(detected_objects) > 0:
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

        # Traffic Light Stopline Collision Points
        if self.stopline_statuses is not None and self.tfl_stoplines is not None:
            for stopline_id, stopline in self.tfl_stoplines.items():
                status = self.stopline_statuses.get(stopline_id, -1)
                if status == 0:
                    if stopline.intersects(path_linestring):
                        intersection_result = stopline.intersection(path_linestring)
                        collision_distance_to_stop = self.braking_safety_distance_stopline

                        # Append traffic light stopline collision point
                        collision_points = np.append(collision_points, np.array(
                            [(intersection_result.x, intersection_result.y, 0.0, 0.0, 0.0, 0.0,
                              collision_distance_to_stop,
                              np.inf,
                              2)],
                            dtype=DTYPE))

        # Braking
        if self.goal_waypoint is not None:
            goal_point_geom = Point(self.goal_waypoint.position.x, self.goal_waypoint.position.y)
            # Buffer a small area around goal point to check intersection with local path buffer
            goal_point_buffer = goal_point_geom.buffer(0.1)
            if local_path_buffer.intersects(goal_point_buffer):
                collision_points = np.append(collision_points, np.array(
                    [(self.goal_waypoint.position.x, self.goal_waypoint.position.y, 0.0, 0.0, 0.0, 0.0,
                      self.braking_safety_distance_goal, np.inf,
                      1)],
                    dtype=DTYPE))


            # Publish collision points or empty if none
            collision_points_msg = msgify(PointCloud2, collision_points)
            collision_points_msg.header = msg.header
            self.local_path_collision_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()

def get_stoplines(lanelet2_map):
    """
    Add all stop lines to a dictionary with stop_line id as key and stop_line as value
    :param lanelet2_map: lanelet2 map
    :return: {stop_line_id: stopline, ...}
    """

    stoplines = {}
    for line in lanelet2_map.lineStringLayer:
        if line.attributes:
            if line.attributes["type"] == "stop_line":
                # add stoline to dictionary and convert it to shapely LineString
                stoplines[line.id] = LineString([(p.x, p.y) for p in line])

    return stoplines


def get_stoplines_trafficlights(lanelet2_map):
    """
    Iterate over all regulatory_elements with subtype traffic light and extract the stoplines and sinals.
    Organize the data into dictionary indexed by stopline id that contains a traffic_light id and the four coners of the traffic light.
    :param lanelet2_map: lanelet2 map
    :return: {stopline_id: {traffic_light_id: {'top_left': [x, y, z], 'top_right': [...], 'bottom_left': [...], 'bottom_right': [...]}, ...}, ...}
    """

    signals = {}

    for reg_el in lanelet2_map.regulatoryElementLayer:
        if reg_el.attributes["subtype"] == "traffic_light":
            # ref_line is the stop line and there is only 1 stopline per traffic light reg_el
            linkId = reg_el.parameters["ref_line"][0].id

            for tfl in reg_el.parameters["refers"]:
                tfl_height = float(tfl.attributes["height"])
                # plId represents the traffic light (pole), one stop line can be associated with multiple traffic lights
                plId = tfl.id

                traffic_light_data = {'top_left': [tfl[0].x, tfl[0].y, tfl[0].z + tfl_height],
                                      'top_right': [tfl[1].x, tfl[1].y, tfl[1].z + tfl_height],
                                      'bottom_left': [tfl[0].x, tfl[0].y, tfl[0].z],
                                      'bottom_right': [tfl[1].x, tfl[1].y, tfl[1].z]}

                # signals is a dictionary indexed by stopline id and contains dictionary of traffic lights indexed by pole id
                # which in turn contains a dictionary of traffic light corners
                signals.setdefault(linkId, {}).setdefault(plId, traffic_light_data)

    return signals

if __name__ == '__main__':
    rospy.init_node('collision_points_manager')
    node = CollisionPointsManager()
    node.run()