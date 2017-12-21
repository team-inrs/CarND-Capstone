#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray, TL_State
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

LIGHT_TO_STOP = 20 # number of waypoints between the tl and the line we need to stop


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # A list of all waypoints
        self.all_waypoints = []
        self.target_velocities = []

        # The car's current position
        self.current_pose = None

        # The index in all_waypoints of the next red traffic light
        self.traffic_light_waypoint_index = -1

        # Subscribe to waypoints and pose
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # Subscribers for /traffic_waypoint and /obstacle_waypoint
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        #Simulator :=> Subscriber for /vehicle/traffic_lights 
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.tl_cb, queue_size=1)

        #for simulator's traffic light
        self.tl_array = []
        self.vb_range = 100 #visibility range when the traffic light is within the sensor's detection range.
        self.pre_dist_tl = float('inf')
        self.pre_ind_tl = -1

        #tl_detector :=> Subscriber for /custom_light 
        rospy.Subscriber('/custom_light', TL_State, self.tl_state_cb, queue_size=1)

        self.next_tl = TrafficLight()
        self.pre_tl_state = TrafficLight.UNKNOWN
        self.yellow_tl_period = 4.0 # USA standard is from 3 to 6 secs, using 4 for conservative driving behavior
        self.yellow_tl_ts = rospy.get_time()
        self.is_yellow_cnter = False;

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
                velocities = self.target_velocities
                if num_points != LOOKAHEAD_WPS: 
                    waypoints = waypoints[closest_waypoint:end_waypoint] + waypoints[:LOOKAHEAD_WPS - num_points]
                    velocities = velocities[closest_waypoint:end_waypoint] + velocities[:LOOKAHEAD_WPS - num_points]
                else: 
                    waypoints = waypoints[closest_waypoint:end_waypoint]
                    velocities = velocities[closest_waypoint:end_waypoint]


                # check the status of the tl
                if self.next_tl.state != TrafficLight.UNKNOWN and self.next_tl.state != TrafficLight.GREEN:
                    # compute the closest waypoint (local) from the traffic lights 
                    closest_distance_tl = float('inf')
                    closest_waypoint_tl = 0
                    for i in range(len(waypoints)):
                        this_distance = self.distance_to_position(waypoints, i, self.all_waypoints[self.traffic_light_waypoint_index].pose.pose.position)
                        if this_distance < closest_distance_tl:
                            closest_distance_tl = this_distance
                            closest_waypoint_tl = i

                    wp_num_to_stop = closest_waypoint_tl - LIGHT_TO_STOP
                    #print "cet_wp",closest_waypoint_tl, "dist", closest_distance_tl,"ind",self.traffic_light_waypoint_index

                    #handle red light, ramp the vel down until it stop at LIGHT_TO_STOP waypoint
                    if self.next_tl.state == TrafficLight.RED:
                        waypoints = self.vel_ramp(wp_num_to_stop, waypoints)

                    else: #handle yellow light
                        # start the timer
                        if not self.is_yellow_cnter:
                            self.is_yellow_cnter = True
                            self.yellow_tl_ts = rospy.get_time()

                        #if the first time detected, be safe to just slow down and stop (no knowledge of how long the yellow has been before detection)
                        if self.pre_tl_state == TrafficLight.UNKNOWN: 
                            self.yellow_tl_ts = -1
                            waypoints = self.vel_ramp(wp_num_to_stop, waypoints)
                        else: 
                            if self.yellow_tl_ts != -1:
                                #compute to see if the car can make it, given the yellow_tl_period
                                #remaining time
                                re_t = self.yellow_tl_period - (rospy.get_time() - self.yellow_tl_ts)
                                if re_t <= 0:
                                    waypoints = self.vel_ramp(wp_num_to_stop, waypoints)
                                else:
                                    #compute the distance to the tl
                                    dll = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
                                    dist = dll(self.current_pose.position,self.next_tl.pose.pose.position)
                                    if (self.current_vel * re_t) <= dist:  # see if the vehicle can make it through yellow
                                        waypoints = self.vel_ramp(wp_num_to_stop, waypoints)
                                    else:
                                        #continue as usual
                                        for i in range(len(waypoints)):
                                            self.set_waypoint_velocity(waypoints, i, velocities[i])
                            else:
                                #continue to slow down
                                waypoints = self.vel_ramp(wp_num_to_stop, waypoints)
                else:
                    #house keeping for yellow light
                    self.is_yellow_cnter = False

                    #move as usual
                    for i in range(len(waypoints)):
                        self.set_waypoint_velocity(waypoints, i, velocities[i])


                #update previous traffic light state
                self.pre_tl_state = self.next_tl.state

                # Publish waypoints
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = waypoints
                self.final_waypoints_pub.publish(lane)

                # Sleep
                rate.sleep()

    def vel_ramp(self, wp_num_to_stop, waypoints):
        if wp_num_to_stop > 0:
            vel_decr = self.current_vel / wp_num_to_stop
            for i in range(len(waypoints)):
                if i < wp_num_to_stop - 3: # one can offset to have sharper ramp
                    vel = (wp_num_to_stop - i - 3) * vel_decr
                    if vel > .05:
                        self.set_waypoint_velocity(waypoints, i, vel)
                        
                        #dll = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
                        #dist = dll(waypoints[i].pose.pose.position,self.next_tl.pose.pose.position)
                        #print "i ",i, "vel",vel,"dist",dist,"wp_num_to_stop", wp_num_to_stop
                    else:
                        self.set_waypoint_velocity(waypoints, i, 0)
                else:
                    self.set_waypoint_velocity(waypoints, i, 0)
        else:
            for i in range(len(waypoints)):
                self.set_waypoint_velocity(waypoints, i, 0)

        return waypoints

    def current_vel_cb(self, current_vel):
        self.current_vel = current_vel.twist.linear.x

    def pose_cb(self, current_pose):
        # Save current pose
        self.current_pose = current_pose.pose

    def waypoints_cb(self, waypoints):
        # Save given waypoints
        self.all_waypoints = waypoints.waypoints

        #record the target velocities to safe time
        for i in range(len(self.all_waypoints)):
            #record waypoint velocity
            self.target_velocities.append(waypoints.waypoints[i].twist.twist.linear.x)

    def traffic_cb(self, msg): #no one is sending this message. 
        self.traffic_light_waypoint_index = msg.data
        #print "traffic waypint index= ", self.traffic_light_waypoint_index

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        pass

        # callback for receiving TL_state message from tl_detector
    def tl_state_cb(self,msg):
        self.next_tl = msg.light

        #used in simulation, with informatoin of all the traffic lights and its status
    def tl_cb(self, msg):
        if self.current_pose == None : return
        
        self.tl_array = msg.lights
        #print "light count = ", len(self.tl_array) # array of all the traffic lights
        dll = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        dist = float("inf")
        ind = 0
        for i in range(len(self.tl_array)):
            if dll(self.current_pose.position,self.tl_array[i].pose.pose.position) < dist :
                dist = dll(self.current_pose.position,self.tl_array[i].pose.pose.position)
                ind = i 

        #only report incoming traffic within visibility range, vb_range
        if dist - self.pre_dist_tl < 0.05 and ind != self.pre_ind_tl and dist < self.vb_range:
            #assign the to the next_tl
            self.next_tl = self.tl_array[ind]
            #print "tl_dist", dist, "state ",self.next_tl.state 

            # compute the traffic_light_waypoint_index
            if len(self.all_waypoints) > 0 and (self.next_tl.state == TrafficLight.YELLOW or self.next_tl.state == TrafficLight.RED):
                closest_distance_tl = float('inf')
                closest_waypoint_tl = 0
                for i in range(len(self.all_waypoints)):
                    this_distance = self.distance_to_position(self.all_waypoints, i, self.next_tl.pose.pose.position)
                    if this_distance < closest_distance_tl:
                        closest_distance_tl = this_distance
                        closest_waypoint_tl = i

                self.traffic_light_waypoint_index = closest_waypoint_tl
            else:
                self.next_tl = TrafficLight()
                self.next_tl.state = TrafficLight.UNKNOWN

        else:
            self.next_tl = TrafficLight()
            self.next_tl.state = TrafficLight.UNKNOWN

        if dist - self.pre_dist_tl > 0.01 and dist < self.vb_range: 
            self.pre_ind_tl = ind

        #update previous distance, used to track incoming traffic light. 
        self.pre_dist_tl = dist

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
