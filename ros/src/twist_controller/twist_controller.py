from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import time
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base,
                steer_ratio, max_lat_accel, max_steer_angle):
        min_speed = 1.0 * ONE_MPH
        self.pid = PID(2.0, 0.4, 0.1)
        self.lpf = LowPassFilter(0.5, 0.02)
        self.yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.last_time = None

    def control(self, proposed_v, proposed_omega, current_v):
        # Return throttle, brake, steer
        if self.last_time is None:
            throttle, brake, steer = 1.0, 0.0, 0.0
        else:
            # get throttle, if negative, change it to break
            throttle = self.pid.step(proposed_v.x - current_v.x, time.time() - self.last_time)
            if throttle < 0: # if we need to decelerate
                brake, throttle = -throttle, 0
            #    brake = self.lpf.filt(brake)
            else:
                brake = 0
            throttle = min(1.0, throttle) # cap between [0, 1]
            steer = self.yaw.get_steering(proposed_v.x, proposed_omega.z, current_v.x)

        self.last_time = time.time() # update time
        # rospy.logwarn('Error:    {}'.format(proposed_v.x - current_v.x))
        # rospy.logwarn('Throttle: {}'.format(throttle))
        # rospy.logwarn('Brake:    {}'.format(brake))
        # rospy.logwarn('Steer:    {}'.format(steer))
        # rospy.logwarn('Speed:    {}'.format(current_v))
        # rospy.logwarn('')
        return throttle, brake, steer
