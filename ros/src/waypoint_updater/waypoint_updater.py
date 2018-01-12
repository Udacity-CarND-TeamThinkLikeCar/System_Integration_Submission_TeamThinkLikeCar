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

        self.waypoints = []
        self.prevFinalWaypoints = []
        self.waypoint_index = 0

        rospy.spin()

    def euclidean_dist(self, waypt1, waypt2):
        return math.sqrt(
            (waypt1.pose.pose.position.x - waypt2.pose.pose.position.x)**2
            + (waypt1.pose.pose.position.y - waypt2.pose.pose.position.y)**2
            + (waypt1.pose.pose.position.z - waypt2.pose.pose.position.z)**2)


    def pose_cb(self, msg):

        lane = Lane()
        lane.header.frame_id = '/finalWayPoints'
        lane.header.stamp = rospy.Time(0)
        limited_waypoints = []

        rospy.loginfo('Current Pos Received as - j1:%s',
                      msg)
	#header: 
	#  seq: 6500
	#  stamp: 
	#    secs: 1515547063
	#    nsecs:  34849882
	#  frame_id: ''
	#level: 2
	#name: "/waypoint_updater"
	#msg: "Current Pos Received as - j1:header: \n  seq: 6780\n  stamp: \n    secs: 1515547063\n\
	#  \    nsecs:  29352903\n  frame_id: \"/world\"\npose: \n  position: \n    x: 1497.886\n\
	#  \    y: 1183.949\n    z: 0.02347874\n  orientation: \n    x: -0.0\n    y: 0.0\n\
	#  \    z: 0.00512600956436\n    w: -0.999986861927"
	#file: "waypoint_updater.py"
	#function: "pose_cb"
	#line: 54

        ####### Need to fill more here


       # if(len(self.prevFinalWaypoints)):
        p = Waypoint()

        p.pose.pose.position.x = msg.pose.position.x
        p.pose.pose.position.y = msg.pose.position.y
        p.pose.pose.position.z = msg.pose.position.z

        shortest_dist = 999
        uptoCount = LOOKAHEAD_WPS # Since we sent 200 pts last time so the nearest pt could be max at 201 distance

        remaining_pts = (len(self.waypoints) - self.waypoint_index)

        if(remaining_pts < uptoCount):
            uptoCount = remaining_pts

        index = self.waypoint_index

        for i in range (self.waypoint_index, (self.waypoint_index + uptoCount)):
            wpdist = self.euclidean_dist(p, self.waypoints[i])

            if(wpdist < shortest_dist):
                index = i # Should Next nearest waypoint index in self.waypoints

        self.waypoint_index = index
        filler_index = self.waypoint_index

        # Fill the waypoints
        for filler_index in range(self.waypoint_index, self.waypoint_index + uptoCount):
            limited_waypoints.append(self.waypoints[filler_index])

        # Fill waypoints upto LOOKAHEAD_WPS, all extra waypoints need to be emplty so car can stop
        if (LOOKAHEAD_WPS > uptoCount):
            for k in range(filler_index, (filler_index + (LOOKAHEAD_WPS - uptoCount))):
                limited_waypoints.append(Waypoint())



        self.prevFinalWaypoints = limited_waypoints
        lane.waypoints = limited_waypoints
        self.final_waypoints_pub.publish(lane)

        pass

    def waypoints_cb(self, Lane):
        self.waypoints = Lane.waypoints
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

