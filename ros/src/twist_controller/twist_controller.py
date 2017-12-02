import rospy
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	# wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle
	self.wheel_base = None
	self.steer_ratio = None
	self.min_speed = None
	self.max_lat_accel = None
	self.max_steer_angle = None
	for key in kwargs:
	    if key == 'wheel_base':
	        self.wheel_base = kwargs[key]
	    elif key == 'steer_ratio':
		self.steer_ratio = kwargs[key]
	    elif key == 'min_speed':
		self.min_speed = kwargs[key]
	    elif key == 'max_lat_accel':
		self.max_lat_accel = kwargs[key]
	    elif key == 'max_steer_angle':
		self.max_steer_angle = kwargs[key]

        #rospy.loginfo("args*:{}".format(args))
        rospy.loginfo("args* kw:{}".format(kwargs))
	
	self.yaw_controller = YawController(self.wheel_base, self.steer_ratio,self. min_speed,self. max_lat_accel, self.max_steer_angle)
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	if kwargs["dbw_enabled"] is False:
	    return 0.,0.,0.
	current_velocity_linear = kwargs["current_velocity_linear"]
	target_velocity_linear = kwargs["target_velocity_linear"]
	target_velocity_angular = kwargs["target_velocity_angular"]	
	steer =self.yaw_controller.get_steering(target_velocity_linear.x, target_velocity_angular.z, current_velocity_linear.x)	
        return 1., 0., steer


