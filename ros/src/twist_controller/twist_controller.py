
import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):

	# Init controllers
    	self.pid = PID(1, 0, 0)
    	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.lowpassFilt = LowPassFilter(0.07, 0.02)

        # Save time for delta time calculation
    	self.previous_time = rospy.get_time()


    def control(self, linear_velocity, angular_velocity, current_linear_velocity, dbw_enabled):

    	# Run controllers
    	steering = 0
    	throttle = 0
    	if dbw_enabled:
            # Get steering only if we are moving forward
            if current_linear_velocity > 0.05:
                #print(linear_velocity)
                steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_linear_velocity)
            else:
                steering = 0

            # Get throttle
            throttle = self.pid.step(linear_velocity - current_linear_velocity, rospy.get_time() - self.previous_time)
            #throttle = self.lowpassFilt.filt(throttle)


        # Save time for delta time calculation
        self.previous_time = rospy.get_time()

        # Calculate braking torque
        brake = 0
        if throttle < 0:
        	brake = -throttle * 1000
        	throttle = 0

        # Cap
        throttle = min(max(throttle, 0.0), 1.0)
        brake = min(max(brake, 0.0), 1.0)

        #print(str(throttle) + ", " + str(brake))

        # Log
        #rospy.loginfo("Target velocity: " + str(linear_velocity) + " Current velocity: " + str(current_linear_velocity) + " Throttle: " + str(throttle) + " Brake: " + str(brake))
        #rospy.loginfo("Angular velocity: " + str(angular_velocity) + " Steering: " + str(steering))

        # Return throttle, brake, steering
        return throttle, brake, steering
"""
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base 	     = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        min_velocity     = kwargs['min_velocity']
        max_lat_accel    = kwargs['max_lat_accel']
        max_steer_angle  = kwargs['max_steer_angle']
        self.decel_limit = kwargs['decel_limit']
        accel_limit      = kwargs['accel_limit']
        self.deadband    = kwargs['deadband']

        self.yawController = YawController(wheel_base, self.steer_ratio, min_velocity, max_lat_accel, max_steer_angle)
        #self.correcting_pid = PID(0.065, 0.000775, 0.775, -1.57, 1.57)
        #self.velocity_pid = PID(0.8, 0.001, 0.2, decel_limit, accel_limit)
        #self.velocity_pid = PID(1.6, 0.00001, 0.04, decel_limit, accel_limit)  Good: best so far
        #self.velocity_pid = PID(2.0, 0.4, 0.1, decel_limit, accel_limit) Too much bias
        #self.velocity_pid = PID(0.1, 0.001, 1.0, decel_limit, accel_limit) Not responsive enough
        #self.velocity_pid = PID(0.9, 0.0005, 0.07, decel_limit, accel_limit) fair
        #self.velocity_pid = PID(5.0, 0.00001, 0.2, decel_limit, accel_limit) ok
        #self.velocity_pid = PID(2.0, 0.0, 0.05, decel_limit, accel_limit) not as good as 1.6
        #self.velocity_pid = PID(2.0, 0.00001, 0.04, decel_limit, accel_limit) good
        #self.velocity_pid = PID(0.35, 0., 0., decel_limit, accel_limit)
        self.velocity_pidHI = PID(0.35, 0., 0., self.decel_limit, accel_limit)
        self.velocity_pidLO = PID(0.7, 0., 0., self.decel_limit/2., accel_limit)
        #self.lowpassFilt = LowPassFilter(accel_limit/2., 0.02) # good
        self.lowpassFilt = LowPassFilter(0.07, 0.02)
        self.topVelocity = 0.

    def control(self, linear_velocity_target, angular_velocity_target, linear_velocity_current):
        # Throttle
        if linear_velocity_target > 2.:
            throttle_correction = self.velocity_pidHI.step(linear_velocity_target-linear_velocity_current, 0.02)
        else:
            throttle_correction = self.velocity_pidLO.step(linear_velocity_target-linear_velocity_current, 0.02)
        throttle_correction = self.lowpassFilt.filt(throttle_correction)
        if throttle_correction > 0.:
            brake = 0.0
            if linear_velocity_target <= 0.01:
                throttle = 0.
                self.velocity_pidLO.reset() # <-- Reset the velocity PID
                self.velocity_pidHI.reset()
            else:
                throttle = throttle_correction
        # Braking
        else:
            throttle = 0.
            brake = -throttle_correction
            if brake < self.deadband * self.decel_limit/2.:
                brake = 0

        # Steering
        if linear_velocity_target > self.topVelocity: # mitigate rapid turning
            self.topVelocity = linear_velocity_target
        if linear_velocity_current > 0.05:
            steering = self.yawController.get_steering(self.topVelocity, angular_velocity_target, linear_velocity_current) # self.correcting_pid.step(angular_velocity_target, 0.1)
        else:
            steering = 0.
        # Alternate approach: steering = angular_velocity_target * self.steer_ratio
        return throttle, brake, steering
    def reset(self):
        self.velocity_pidLO.reset()
        self.velocity_pidHI.reset()
"""
