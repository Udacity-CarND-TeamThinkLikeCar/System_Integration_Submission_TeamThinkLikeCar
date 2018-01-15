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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

    #    rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
    #    rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb) // Assuming Int32 here

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.rxd_lane_obj = []
        self.prevFinalWaypoints = []
        self.waypoint_index = 0

        rospy.spin()

    def euclidean_dist(self, waypt1, waypt2):
        return math.sqrt(
            (waypt1.pose.pose.position.x - waypt2.pose.pose.position.x)**2
            + (waypt1.pose.pose.position.y - waypt2.pose.pose.position.y)**2
            + (waypt1.pose.pose.position.z - waypt2.pose.pose.position.z)**2)


    def pose_cb(self, msg):

        limited_waypoints = []

        rospy.loginfo('Current Pos Received as - j1:%s', msg)

        p = Waypoint()

        p.pose.pose.position.x = msg.pose.position.x
        p.pose.pose.position.y = msg.pose.position.y
        p.pose.pose.position.z = msg.pose.position.z

        shortest_dist = 99999
        uptoCount = LOOKAHEAD_WPS # Since we sent 200 pts last time so the nearest pt could be max at 200 distance

        remaining_pts = (len(self.rxd_lane_obj.waypoints) - self.waypoint_index)

        if(remaining_pts < uptoCount):
            uptoCount = remaining_pts

        index = self.waypoint_index

        for i in range (self.waypoint_index, (self.waypoint_index + uptoCount)):
            wpdist = self.euclidean_dist(p, self.rxd_lane_obj.waypoints[i])
            if(wpdist < shortest_dist):
                shortest_dist = wpdist
                index = i # Should Next nearest waypoint index in self.rxd_lane_obj.waypoints

        self.waypoint_index = index
        filler_index = self.waypoint_index

        # Fill the waypoints
        for filler_index in range(self.waypoint_index, self.waypoint_index + uptoCount):
            limited_waypoints.append(self.rxd_lane_obj.waypoints[filler_index])

        # Fill waypoints upto LOOKAHEAD_WPS, all extra waypoints need to be emplty so car can stop
        if (LOOKAHEAD_WPS > uptoCount):
            for k in range(filler_index, (filler_index + (LOOKAHEAD_WPS - uptoCount))):
                extraWp = Waypoint()
                extraWp.twist.twist.linear.x = 0 # 0 velocity
                limited_waypoints.append(extraWp)

        self.prevFinalWaypoints = limited_waypoints

        lane = Lane()
        lane.header = self.rxd_lane_obj.header
        lane.waypoints = limited_waypoints

    #    rospy.loginfo('================ limited_waypoints================================================== - j1:%s',
     #                 lane.waypoints)
        rospy.loginfo('+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
        self.final_waypoints_pub.publish(lane)

        pass

    def waypoints_cb(self, lane):
        self.rxd_lane_obj = lane
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
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

