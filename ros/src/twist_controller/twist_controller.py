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
        # self.pid = PID(2.0, 0.4, 0.1)
        self.pid = PID(0.3, 0.003, 4.0)
        self.lpf = LowPassFilter(0.5, 0.02)
        self.yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.v_mass = vehicle_mass
        self.w_radius = wheel_radius
        self.d_limit = decel_limit
        self.last_time = None

    def control(self, proposed_v, proposed_omega, current_v):
        # Return throttle, brake, steer
        if self.last_time is None:
            throttle, brake, steer = 1.0, 0.0, 0.0
        else:
            dt = time.time() - self.last_time
            # get throttle, if negative, change it to break
            error = proposed_v.x - current_v.x
            throttle = self.pid.step(error, time.time() - self.last_time)
            throttle = max(0.0, min(1.0, throttle))

            if error < 0: # if we need to decelerate
                deceleration = abs(error) / 50
                if abs(deceleration ) > abs(self.d_limit) / 50:
                    deceleration = self.d_limit / 50
                longitudinal_force = self.v_mass * deceleration
                brake = longitudinal_force * self.w_radius

                throttle = 0
            else:
                brake = 0
            steer = self.yaw.get_steering(proposed_v.x, proposed_omega.z, current_v.x)

        self.last_time = time.time() # update time
        # rospy.logwarn('proposed v:    {}'.format(proposed_v.x))
        # rospy.logwarn('current v:    {}'.format(current_v.x))
        # rospy.logwarn('Error:    {}'.format(proposed_v.x - current_v.x))
        # rospy.logwarn('Throttle: {}'.format(throttle))
        # rospy.logwarn('Brake:    {}'.format(brake))
        # rospy.logwarn('Steer:    {}'.format(steer))
        # rospy.logwarn('Speed:    {}'.format(current_v))
        # rospy.logwarn('')
        return throttle, brake, steer
