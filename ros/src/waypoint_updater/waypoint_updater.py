#!/usr/bin/env python

import rospy
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
        self.prevFinalWaypoints = []
        self.waypoint_index = 0
        self.numOfWaypoints = 0
        self.is_stop_req = 1
        self.stop_wayp_index = 999999 # Default very high number
        self.decrement_factor = 50 # We will try to start decrementing speed from these many way points

        rospy.spin()

    @staticmethod
    def euclidean_dist(p1, p2):
        x, y, z = p1.pose.pose.position.x - p2.pose.pose.position.x, \
                  p1.pose.pose.position.y - p2.pose.pose.position.y, \
                  p1.pose.pose.position.z - p2.pose.pose.position.z
        return math.sqrt(x * x + y * y + z * z)

    def pose_cb(self, msg):

        limited_waypoints = []

        rospy.loginfo('Current Pos Received as - j1:%s', msg)

        p = Waypoint()

        p.pose.pose.position.x = msg.pose.position.x
        p.pose.pose.position.y = msg.pose.position.y
        p.pose.pose.position.z = msg.pose.position.z

        shortest_dist = 99999
        uptoCount = LOOKAHEAD_WPS  # Since we sent 200 pts last time so the nearest pt could be max at 200 distance

        remaining_pts = (self.numOfWaypoints - self.waypoint_index + 1)

        if remaining_pts > 0:
            if remaining_pts < uptoCount:
                uptoCount = remaining_pts

            index = self.waypoint_index
            foundIndexCount = 0

            for i in range(self.waypoint_index, (self.waypoint_index + uptoCount)):
                wpdist = self.euclidean_dist(p, self.rxd_lane_obj.waypoints[i])
                if wpdist < shortest_dist:
                    shortest_dist = wpdist
                    foundIndexCount = 0
                    index = i  # Should be next nearest waypoint index in self.rxd_lane_obj.waypoints
                if wpdist > shortest_dist:
                    foundIndexCount += 1
                if foundIndexCount > 100:  # If distance is increasing, means we found it
                    rospy.loginfo('+++++++++++++++ Breaking loop at index  - j1:%s', i)
                    break

            self.waypoint_index = index
            filler_index = 0

            # Fill the waypoints
            for count_index in range(self.waypoint_index, self.waypoint_index + uptoCount - 1):
                limited_waypoints.append(self.rxd_lane_obj.waypoints[count_index])
                filler_index = count_index

            inrange = 0
            if self.waypoint_index < self.stop_wayp_index and (self.stop_wayp_index - self.waypoint_index < uptoCount):
                inrange = 1

            if self.is_stop_req == 1 and inrange == 1:
                curr_stop_index = self.stop_wayp_index - self.waypoint_index
                limited_waypoints = self.set_group_velocity(limited_waypoints, self.decrement_factor, curr_stop_index)


            # Fill waypoints upto LOOKAHEAD_WPS, all extra waypoints need to be empty so car can stop
            # if LOOKAHEAD_WPS > uptoCount:
            #     for k in range(filler_index, (filler_index + (LOOKAHEAD_WPS - uptoCount))):
            #         extraWp = Waypoint()
            #         extraWp.twist.twist.linear.x = 0  # 0 velocity
            #         limited_waypoints.append(extraWp)

        self.prevFinalWaypoints = limited_waypoints

        lane = Lane()
        lane.header = self.rxd_lane_obj.header
        lane.waypoints = limited_waypoints

        rospy.loginfo('++++++++++++++++++ Broadcasting /final_waypoints +++++++++++++++++++')
        self.final_waypoints_pub.publish(lane)

        pass

    def set_group_velocity(self, limited_waypoints, decrement_factor, curr_stop_index):

        if curr_stop_index < decrement_factor:
            decrement_factor = curr_stop_index

        start_dec_index = curr_stop_index - decrement_factor # 0 when stop index < 50

        # Decreasing velocity gracefully,
        for i in range(0, len(limited_waypoints)):
            if start_dec_index < i < curr_stop_index:
                limited_waypoints[i].twist.twist.linear.x = decrement_factor
                decrement_factor -= 1
            elif i > curr_stop_index:
                limited_waypoints[i].twist.twist.linear.x = 0

        return limited_waypoints

    def waypoints_cb(self, lane):
        self.rxd_lane_obj = lane
        self.numOfWaypoints = len(self.rxd_lane_obj.waypoints)
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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
