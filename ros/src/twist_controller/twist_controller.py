from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, dbw_frequency):

        # Set up controller for steering with min_speed 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Set up controller for throttle
        # Parameters were determined experimental
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0    # Minimum throttle value
        mx = 0.2    # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Set up low pass filter
        # It is filtering out the high frequency noise of the current_vel
        tau = 0.5   # 1/(2pi * tau) = cutoff frequency
        ts = 1./dbw_frequency   # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, linear_vel, angular_vel, dbw_enabled):
        
        # Avoid accumulating error when drive-by-wire is turned off which would
        # happen when we have a PID controller with an interval term keep running
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            # If the car should stop we hit the brake
            throttle = 0
            brake = 700 # Nm - to hold the car in place if we are stopped at a light
        elif throttle < .1 and vel_error < 0:
            # If we are going faster than we want to we should decelerate
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Torque Nm

        # Return throttle, brake, steer
        return throttle, brake, steering
