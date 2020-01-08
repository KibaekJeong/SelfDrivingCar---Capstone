import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
    	accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        self.throttle_controller = PID(kp=0.3, ki=0.0, kd=0.01, mn=decel_limit, mx=0.5*accel_limit)
        self.brake_controller = PID(kp=100,ki=0.0,kd=1.0,mn=0.0,mx=3000)

        tau = 0.5 # 1/(2pi*tau) = cutoff freq (?)
        ts = .02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        brake = 0.0
        vel_error = 0.0

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

        if vel_error>0: # if slower than target speed, throttle
            throttle = self.throttle_controller.step(vel_error,sample_time)
            brake= 0.0
            self.brake_controller.reset()
        else: # if faster than target speed, brake
            brake = self.brake_controller.step(-1.*vel_error,sample_time)
            throttle = 0.0
            self.throttle_controller.reset()

        if linear_vel == 0. and current_vel < 0.1:
        	throttle = 0.0
        	brake = 700 # N*m the torqe to hold carla

        return throttle, brake, steering
