import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import Twist
import rclpy.subscription
from sensor_msgs.msg import JointState

# from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from diff_drive_simple.cfg import DiffDriveSimpleConfig

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

def map(v, in_min, in_max, out_min, out_max):
	v = clamp(v, in_min, in_max)
	return (v - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

class DiffDrive(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        # Declare parameters with default values
        self.declare_parameter('min_speed', 0.0)  
        self.declare_parameter('max_speed', 1.0)

        # Get parameter values
        self.SPEED_MIN = self.get_parameter('min_speed').value
        self.SPEED_MAX = self.get_parameter('max_speed').value

        self.cmdsub = self.create_subscription(Twist, "cmd_vel", self.velocity, 10)
        self.wheel_pub = self.create_publisher(JointState, "diff_drive", 1)

        # TODO:
        """
        self.reconfigure_server = DynamicReconfigureServer(DiffDriveSimpleConfig, self.dynamic_reconfigure_callback)
        """

        self.get_logger().info('Diff Drive Ready')

    def dynamic_reconfigure_callback(self, config, level):

        if config.min_speed > 0.0 and config.min_speed <= self.SPEED_MAX:
            self.SPEED_MIN = config.min_speed
            
        if config.max_speed >= self.SPEED_MIN:
            self.SPEED_MAX = config.max_speed

        config.min_speed = self.SPEED_MIN
        config.max_speed = self.SPEED_MAX

        self.get_logger().info(f'Diff Drive reconfigured: Min: {self.SPEED_MIN} Max: {self.SPEED_MAX}')

        return config
        
    def velocity(self, msg):

        if math.isnan(msg.angular.z) or math.isnan(msg.linear.x):
            self.get_logger().fatal(f'Warning: NaN value detected in velocity: angular.z={msg.angular.z}, linear.x={msg.linear.x}')
            return
        
        left_vel = 0.0
        right_vel = 0.0

        if msg.angular.z != 0 or msg.linear.x != 0:
            
            #consistency with diff_drive_controller, which does not reverse correctly
            #fix the angular inversion in the twist publisher, since it's the only way to make it compatible with both
            if msg.linear.x < 0:
                msg.angular.z = -msg.angular.z

            left, right = self.diffdrive(msg.angular.z, msg.linear.x)

            if math.isnan(left) or math.isnan(right):
                self.get_logger().fatal(f'Warning: NaN value detected in diff_drive: left={left}, right={right}')
                return
            
            left_spd = clamp(abs(left), self.SPEED_MIN, self.SPEED_MAX)
            right_spd = clamp(abs(right), self.SPEED_MIN, self.SPEED_MAX)

            left_vel = math.copysign(left_spd, left)
            right_vel = math.copysign(right_spd, right)

        state = JointState()
        state.name = ["left_wheel", "right_wheel"]
        state.velocity = [left_vel, right_vel]
        self.wheel_pub.publish(state)

    def diffdrive(self, x,  y):

        x = clamp(x, -self.SPEED_MAX, self.SPEED_MAX)
        y = clamp(y, -self.SPEED_MAX, self.SPEED_MAX)

        # First Compute the angle in deg
        # First hypotenuse
        z = math.sqrt(x * x + y * y)

        # angle in radians
        if z == 0:
            rad = 0
        else:
            rad = math.acos(math.fabs(x) / z)

        # and in degrees
        angle = rad * 180 / math.pi

        # Now angle indicates the measure of turn
        # along a straight line, with an angle 0, the turn co-efficient is same
        # this applies for angles between 0-90, with angle 0 the coeff is -1
        # with angle 45, the co-efficient is 0 and with angle 90, it is 1

        tcoeff = -1 + (angle / 90) * 2
        turn = tcoeff * math.fabs(math.fabs(y) - math.fabs(x))
        turn = round(turn * 100, 0) / 100

        # And max of y or x is the movement
        mov = max(math.fabs(y), math.fabs(x))

        # First and third quadrant
        if (x >= 0 and y >= 0) or (x < 0 and y < 0):
            rawLeft = mov
            rawRight = turn

        else:
            rawRight = mov
            rawLeft = turn

        if y < 0:
            return [-rawLeft, -rawRight]
        
        return [rawRight, rawLeft]

def main(args=None):
    rclpy.init(args=args)

    diff_drive_node = DiffDrive()
    rclpy.spin(diff_drive_node)

    diff_drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
