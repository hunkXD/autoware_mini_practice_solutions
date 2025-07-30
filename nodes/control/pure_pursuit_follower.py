#! /usr/bin/env python3

import rospy

from autoware_mini.msg import Path, VehicleCmd
from geometry_msgs.msg import PoseStamped

from shapely.geometry import LineString, Point
from shapely import prepare, distance

from tf.transformations import euler_from_quaternion
import numpy as np

from scipy.interpolate import interp1d

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        # Reading in the parameter values
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")

        # Create class variable
        self.path_linestring = None

        self.distance_to_velocity_interpolator = None

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        # TODO
        # convert waypoints to shapely linestring
        self.path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(self.path_linestring)

        # collect waypoint x and y coordinates
        waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
        # Calculate distances between points
        distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0) ** 2, axis=1)))
        # add 0 distance in the beginning
        distances = np.insert(distances, 0, 0)
        # Extract velocity values at waypoints
        velocities = np.array([w.speed for w in msg.waypoints])

        # Create a distance-to-velocity interpolator for the path
        self.distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear', bounds_error = True)



    def current_pose_callback(self, msg):

        # TODO
        # Create and fill vehicle current_pose message
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = msg.header.stamp

        if self.path_linestring is None:
            return

        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        d_ego_from_path_start = self.path_linestring.project(current_pose)

        # using euler_from_quaternion to get the heading angle
        _, _, heading = euler_from_quaternion(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)

        alpha = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x) - heading

        ld = distance(current_pose, lookahead_point)

        # delta nominator
        steering_angle = np.arctan((2 * self.wheel_base * np.sin(alpha)) / ld)
        velocity = self.distance_to_velocity_interpolator(d_ego_from_path_start)

        vehicle_cmd_msg = VehicleCmd()
        vehicle_cmd_msg.ctrl_cmd.steering_angle = steering_angle
        vehicle_cmd_msg.ctrl_cmd.linear_velocity = velocity

        # Create and fill vehicle command message
        vehicle_cmd_msg.header.stamp = current_pose_msg.header.stamp
        vehicle_cmd_msg.header.frame_id = "base_link"



        # Publish the message
        self.vehicle_cmd_pub.publish(vehicle_cmd_msg)


        #  self.current_pose = msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
