#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5
MAX_ACCEL = 1.


class WaypointUpdater(object):
    def __init__(self):
        rospy.logwarn('# WaypointUpdater Initialization')
        rospy.init_node('waypoint_updater')

        # Initialize variables
        self.base_waypoints = None
        self.num_base_waypoints = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.update_rate = rospy.get_param('~update_rate', 50)
        self.current_velocity = None
        self.max_velocity = rospy.get_param('/waypoint_loader/velocity') / 3.6

        # Set up subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Set up publisher for future waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Replaces rospy.spin() to have more control over the publishing frequency
        self.loop()

    def loop(self):
        # Set rate to frequency in Hz
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                self.publish_final_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # Get coordinates of the car
        pos = [self.pose.pose.position.x, self.pose.pose.position.y]

        # Using KDTree to get the closest point to that position
        closest_idx = self.waypoint_tree.query(pos, 1)[1]

        # Check if closest is ahead or behind vehicle
        # Equation for hyperplane through closest_coords
        closest_vect = np.array(self.waypoints_2d[closest_idx])
        prev_vect = np.array(self.waypoints_2d[closest_idx - 1])
        pos_vect = np.array(pos)

        val = np.dot(closest_vect - prev_vect, pos_vect - closest_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % self.num_base_waypoints
        return closest_idx

    def publish_final_waypoints(self):
        final_lane = self.generate_lane()
        # Publish the message on the /final_waypoints topic
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane() 

        # Slice the waypoints from closest to horizon
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            # If no red traffic light, publish the base waypoints
            lane.waypoints = self.accelerate_waypoints(waypoints, closest_idx)
        else:
            lane.waypoints = self.decelerate_waypoints(waypoints, closest_idx)

        return lane

    def accelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            # Create new waypoint message to not override the base_waypoints waypoints as
            # they are only read once
            p = Waypoint()
            p.pose = wp.pose
            dist = self.distance(waypoints, i, LOOKAHEAD_WPS - 1)
            vel = self.current_velocity + math.sqrt(2 * MAX_ACCEL * dist)

            # Check whether speed is higher than max_velocity
            if vel > self.max_velocity:
                vel = self.max_velocity
            
            p.twist.twist.linear.x = self.max_velocity
            temp.append(p)
        return temp

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            # Create new waypoint message to not override the base_waypoints waypoints as 
            # they are only read once
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from the line so front of car stops at line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            # rospy.logwarn("{}# Distance: {}; dl: {}; wp1: {}".format(i, dist, dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position), wp1))
            # Set wp1 to current i to add up distance between this point and next point in the next cycle
            wp1 = i
        return dist

# Callbacks for subscribers
    def pose_cb(self, msg):
        # Store the pose
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # Store the base_waypoints in the object
        self.base_waypoints = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.num_base_waypoints = len(self.waypoints_2d)
            # KDTree is a data structure allowing to look up the closest point in space very efficiently
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity  = msg.twist.linear.x

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
