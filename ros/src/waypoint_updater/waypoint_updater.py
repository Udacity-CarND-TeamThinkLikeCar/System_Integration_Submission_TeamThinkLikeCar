#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #    rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb) // Assuming Int32 here

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.rxd_lane_obj = []
        self.car_pos_index = 0
        self.numOfWaypoints = 0
        self.is_stop_req = 0
        self.short_of_points = 0
        self.stop_wayp_index = 9999999  # Default very high number
        self.decrement_factor = 79  # We will try to start decrementing speed from these many way points
        self.velocity_array = []

        rospy.spin()

    @staticmethod
    def euclidean_dist(p1, p2):
        x, y, z = p1.pose.pose.position.x - p2.pose.pose.position.x, \
                  p1.pose.pose.position.y - p2.pose.pose.position.y, \
                  p1.pose.pose.position.z - p2.pose.pose.position.z
        return math.sqrt(x * x + y * y + z * z)

    def pose_cb(self, msg):

        limited_waypoints = []

        p = Waypoint()

        p.pose.pose.position.x = msg.pose.position.x
        p.pose.pose.position.y = msg.pose.position.y
        p.pose.pose.position.z = msg.pose.position.z

        shortest_dist = 9999999  # Default very high number
        uptoCount = LOOKAHEAD_WPS  # Since we sent 200 pts last time so the nearest pt could be max at 200 distance

        remaining_pts = (self.numOfWaypoints - self.car_pos_index + 1)

        if remaining_pts > 0:
            if remaining_pts < uptoCount:
                uptoCount = remaining_pts
                if uptoCount < 80:
                    self.short_of_points = 1
                    self.stop_wayp_index = self.numOfWaypoints - 1   # The last known index

            index = self.car_pos_index
            foundIndexCount = 0

            for i in range(self.car_pos_index, self.numOfWaypoints):  #(self.car_pos_index + uptoCount + 200)):
                wpdist = self.euclidean_dist(p, self.rxd_lane_obj.waypoints[i])

                if wpdist < shortest_dist:
                    shortest_dist = wpdist
                    foundIndexCount = 0
                    index = i  # Should be next nearest waypoint index in self.rxd_lane_obj.waypoints

                if wpdist > shortest_dist:
                    foundIndexCount += 1

                if foundIndexCount > 150:  # If distance is increasing, means we found it
                    break

            self.car_pos_index = index

            # Fill the waypoints
            for count_index in range(self.car_pos_index, self.car_pos_index + uptoCount):
                limited_waypoints.append(self.rxd_lane_obj.waypoints[count_index])

       #     rospy.loginfo('++++++++++++++++ self.car_pos_index : %d ', self.car_pos_index )
       #     rospy.loginfo('++++++++++++++++  self.stop_wayp_index %d ' , self.stop_wayp_index)

            inrange = 0

            diff = self.stop_wayp_index - self.car_pos_index

            if self.car_pos_index <= self.stop_wayp_index:
                if diff < uptoCount:
                    inrange = 1

            rospy.loginfo('++++++++++++++++ inrange : %s ', inrange)

            if (self.is_stop_req == 1 and inrange == 1) or self.short_of_points == 1:

                adv_stop_wap = 80
                for i in range(adv_stop_wap):
                    self.velocity_array.append(0)

                curr_stop_index = self.stop_wayp_index - self.car_pos_index
                stop_index_vel = limited_waypoints[curr_stop_index].twist.twist.linear.x

                self.velocity_array[self.decrement_factor] = stop_index_vel
                self.velocity_array[self.decrement_factor - 1] = stop_index_vel - 2
                self.velocity_array[self.decrement_factor - 2] = stop_index_vel - 3
                self.velocity_array[self.decrement_factor - 3] = stop_index_vel - 4
                self.velocity_array[self.decrement_factor - 4] = 1

                limited_waypoints = self.prepare_to_stop(limited_waypoints, self.decrement_factor, curr_stop_index)

        # Prepare to broadcast
        lane = Lane()
        lane.header = self.rxd_lane_obj.header
        lane.waypoints = limited_waypoints

        self.final_waypoints_pub.publish(lane)
        pass

    def prepare_to_stop(self, limited_waypoints, decrement_factor, curr_stop_index):

        if curr_stop_index < decrement_factor:
            decrement_factor = curr_stop_index

        start_dec_index = curr_stop_index - decrement_factor  # 0 when stop index < 80

        for i in range(0, len(limited_waypoints)):
            if start_dec_index <= i <= curr_stop_index:
                limited_waypoints[i].twist.twist.linear.x = self.velocity_array[decrement_factor]
                decrement_factor -= 1
            elif i > curr_stop_index:
                limited_waypoints[i].twist.twist.linear.x = 0

        return limited_waypoints

    def waypoints_cb(self, lane):
        self.rxd_lane_obj = lane
        self.numOfWaypoints = len(self.rxd_lane_obj.waypoints)
        # Uncomment this, if you want to test on short track
        #self.numOfWaypoints = self.numOfWaypoints // 30
        pass

    def traffic_cb(self, wp_index):
        if wp_index.data == -1:
            self.is_stop_req = 0
            self.stop_wayp_index = 9999999
        elif wp_index.data > self.car_pos_index:
            self.is_stop_req = 1
            self.stop_wayp_index = wp_index.data

        rospy.loginfo('+++++TTTTT self.stop_wayp_index : %s', wp_index.data)

        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
