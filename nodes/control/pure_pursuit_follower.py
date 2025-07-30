#! /usr/bin/env python3

import rospy

from autoware_mini.msg import Path, VehicleCmd
from geometry_msgs.msg import PoseStamped

class PurePursuitFollower:
    def __init__(self):

        # Parameters

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)
        rospy.Subscriber('/control/vehicle_cmd', VehicleCmd, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        # TODO
        pass

    def current_pose_callback(self, msg):
        # TODO
        current_pose = PoseStamped()
        current_pose.header.stamp = msg.header.stamp

        self.current_pose_pub.publish(current_pose)

        vehicle_cmd = VehicleCmd()
        vehicle_cmd.ctrl_cmd.steering_angle = 0.2
        vehicle_cmd.ctrl_cmd.linear_velocity = 10.0

        # Create and fill vehicle command message
        vehicle_cmd.header.stamp = current_pose.header.stamp
        vehicle_cmd.header.frame_id = "base_link"

        # Publish the message
        self.vehicle_cmd_pub.publish(vehicle_cmd)

        #  self.current_pose = msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()
