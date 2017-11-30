#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
			
	#self.current_pose = msg
	waypoints_ahead = Lane()
	waypoints_ahead.header = msg.header

	if len(self.waypoints) > 0:
	    for waypoint in self.waypoints:
		if waypoint.pose.pose.position.x > msg.pose.position.x and \
                   len(waypoints_ahead.waypoints)< LOOKAHEAD_WPS:
		    waypoints_ahead.waypoints.append(waypoint)
            rospy.loginfo("my waypoint ahead:{}".format(waypoints_ahead.waypoints[199].pose.pose.position.x))
            rospy.loginfo("my waypoint ahead length:{}".format(len(waypoints_ahead.waypoints)))	
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	for waypoint in waypoints.waypoints:
	   position = waypoint.pose.pose.position
	   orientation = waypoint.pose.pose.orientation
           rospy.loginfo("x:{0}, y:{1}, z:{2}".format(position.x, position.y, position.z))
	self.waypoints = waypoints.waypoints
	rospy.loginfo("waypoints length {0}".format(len(self.waypoints)))
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
