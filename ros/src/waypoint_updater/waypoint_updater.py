#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
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
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish.
#LOOKAHEAD_WPS = 25 # Number of waypoints we will publish.

LIGHT_TO_STOP = 3 # number of waypoints between the tl and the line we need to stop


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # A list of all waypoints
        self.all_waypoints = []

        self.current_velocity = 0
        self.target_velocity = 0
        self.stopping_velocities = []
        self.num_to_stop = 100

        # The car's current position
        self.current_pose = None

        # The index in all_waypoints of the next red traffic light
        self.traffic_light_waypoint_index = -1

        # Subscribe to waypoints and pose
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # Subscribers for /traffic_waypoint and /obstacle_waypoint
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        # Subscriber for current velocity
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_vel_cb)

        # Publish final_waypoints
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Create waypoints Lane for publishing
        lane = Lane()
        lane.header.frame_id = '/world'

        # Start loop, 10 times a second
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # wait to move until the traffic light has been discovered
            #if self.traffic_light_waypoint_index != -1:

            # wait until we have knowledge of the car status and track
            if len(self.all_waypoints) > 0 and self.current_pose != None:
                # Get waypoints
                waypoints = self.all_waypoints

                # Find the nearest waypoint to where we currently are
                position = self.current_pose.position
                closest_distance = float('inf')
                closest_waypoint = 0
                for i in range(len(waypoints)):
                    this_distance = self.distance_to_position(waypoints, i, position)
                    if this_distance < closest_distance:
                        closest_distance = this_distance
                        closest_waypoint = i

                # Cut waypoints from closest_waypoint to LOOKAHEAD_WPS or end of list (if end of list loop back around to start)
                end_waypoint = min(len(waypoints), closest_waypoint + LOOKAHEAD_WPS)
                num_points = end_waypoint-closest_waypoint
                if num_points != LOOKAHEAD_WPS: waypoints = waypoints[closest_waypoint:end_waypoint] + waypoints[:LOOKAHEAD_WPS - num_points]
                else: waypoints = waypoints[closest_waypoint:end_waypoint]


                #EVERYTHING UP TO PUBLISH MAY BE GARBAGE

                # get the number of waypoints between where we currently are and where we need to stop
                wp_num_to_stop = self.traffic_light_waypoint_index - closest_waypoint - LIGHT_TO_STOP

                # Only change waypoint velocities if we are close enough to slow down and are in front of a red light
                if wp_num_to_stop <= self.num_to_stop and 0 <= wp_num_to_stop:
                    #print('here')
                    # Loop over every waypoint we care about
                    for i in range(len(waypoints)):
                        # Does this waypoint have a non zero velocity?
                        if i < wp_num_to_stop:
                            ind = self.num_to_stop - wp_num_to_stop + i
                            target_vel = self.stopping_velocities[ind]
                            self.set_waypoint_velocity(waypoints, i, target_vel)
                        else:
                            self.set_waypoint_velocity(waypoints, i, 0)
                else:
                    for i in range(len(waypoints)):
                        self.set_waypoint_velocity(waypoints, i, self.target_velocity)

                #print(self.get_waypoint_velocity(waypoints[0]))

                # Publish waypoints
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = waypoints
                self.final_waypoints_pub.publish(lane)

                # Sleep
                rate.sleep()

    def current_vel_cb(self, current_vel):
        self.current_vel = current_vel.twist.linear.x

    def pose_cb(self, current_pose):
        # Save current pose
        self.current_pose = current_pose.pose

    def waypoints_cb(self, waypoints):
        # Save given waypoints
        self.all_waypoints = waypoints.waypoints

        self.target_velocity = waypoints.waypoints[0].twist.twist.linear.x


        vel_diff = self.target_velocity / self.num_to_stop
        for i in range(self.num_to_stop):
            self.stopping_velocities.append((self.target_velocity-i*vel_diff))

    def traffic_cb(self, msg):
        self.traffic_light_waypoint_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
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

    def distance_to_position(self, waypoints, wp, position):
        calculate_distance = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        distance = calculate_distance(waypoints[wp].pose.pose.position, position)
        return distance

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
