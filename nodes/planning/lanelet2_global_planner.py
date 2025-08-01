#! /usr/bin/env python3

# All these imports from lanelet2 library should be sufficient
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
import rospy

from shapely.geometry import Point, LineString
from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Waypoint, Path, VehicleCmd
from shapely import distance



class Lanelet2GlobalPlanner:
    def __init__(self):

        # Parameters
        """
        Load a lanelet2 map from a file and return it
        :param lanelet2_map_path: name of the lanelet2 map file
        :param coordinate_transformer: coordinate transformer
        :param use_custom_origin: use custom origin
        :param utm_origin_lat: utm origin latitude
        :param utm_origin_lon: utm origin longitude
        :return: lanelet2 map
        """

        # get parameters
        self.coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        self.use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        self.utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Reading in the parameter values
        self.lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.speed_limit = rospy.get_param("~speed_limit")
        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")
        self.distance_to_goal_limit = rospy.get_param("lanelet2_global_planner/distance_to_goal_limit")

        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")

        # Create class variable
        self.lanelet2_map = self.load_lanelet2_map(self.lanelet2_map_path)

        self.current_location = None

        self.goal_point = None

        self.graph = None

        self.waypoints = None

        self.current_speed = None



        # internal variables
        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Path, queue_size=10)
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        # rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_point_callback, queue_size=1)

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # Assumes that the goal point is stored
        if self.goal_point is not None:
            # Convert to Shapely points
            current_point = Point(self.current_location.x, self.current_location.y)
            goal_point = Point(self.goal_point.x, self.goal_point.y)

            remaining_dist = float(current_point.distance(goal_point))
            if (remaining_dist < self.distance_to_goal_limit):
                self.goal_point = None
                self.publish_waypoints([])
                rospy.loginfo("GOAL HAS BEEN REACHED")


    def goal_point_callback(self, msg):

        # log info
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                      msg.pose.orientation.w, msg.header.frame_id)

        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # get start and end lanelets
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
        # find routing graph
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)

        if route is None:
           rospy.logwarn("No route found after entered goal point! Try again.")
           return

        # find shortest path
        path = route.shortestPath()
        # This returns LaneletSequence to a point where a lane change would be necessary to continue
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        # Convert lanelet path to waypoints
        self.waypoints = self.convert_2_waypoints(path_no_lane_change)

        # Publish the global path
        self.publish_waypoints(self.waypoints)

    def convert_2_waypoints(self, path):
        waypoints = []
        for lanelet in path:
            # code to check if lanelet has attribute speed_ref
            if 'speed_ref' in lanelet.attributes:
                speed = float(lanelet.attributes['speed_ref']) / 3.6
            else:
                speed = float(self.speed_limit) / 3.6

            # create Waypoint (from autoware_mini.msgs import Waypoint) and get the coordinats from lanelet.centerline points
            for point in lanelet.centerline:
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                waypoints.append(waypoint)

        return waypoints

    def publish_waypoints(self, waypoints):

        path = Path()
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints
        self.waypoints_pub.publish(path)

    def load_lanelet2_map(self, lanelet2_map_path):

        # Load the map using Lanelet2
        if self.coordinate_transformer == "utm":
            projector = UtmProjector(Origin(self.utm_origin_lat, self.utm_origin_lon), self.use_custom_origin, False)
        else:
            raise ValueError(
                'Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): ' + self.coordinate_transformer)

        lanelet2_map = load(lanelet2_map_path, projector)

        return lanelet2_map

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner', log_level=rospy.INFO)
    node = Lanelet2GlobalPlanner()
    node.run()

