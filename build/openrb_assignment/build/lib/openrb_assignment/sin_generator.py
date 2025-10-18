import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from math import sin
from math import pi
from time import time 

class SinGenerator(Node):

    def __init__(self):
        super().__init__('sin_generator')
        #make pub
        self.publisher_ = self.create_publisher(Float64, 'sinSignal', 10)
        #declaring params
        self.declare_parameter('frequency', 1.0) #hertz
        self.declare_parameter('amplitude', -20.0) 
        self.declare_parameter('offset', 0.0) #degrees
        self.declare_parameter('timer_period',0.1)
        #extracting params
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value # seconds
        self.freq= self.get_parameter('frequency').get_parameter_value().double_value 
        self.amp = self.get_parameter('amplitude').get_parameter_value().double_value 
        self.off = self.get_parameter('offset').get_parameter_value().double_value 
        #create timer and counter
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.count = 0
       

    def timer_callback(self):
        passed_time = (self.get_clock().now() - self.start_time).nanoseconds*1e-9 #seconds
        msg = Float64()
        msg.data = self.amp*sin(2*pi*self.freq*passed_time + (self.off*pi)/180)
        self.publisher_.publish(msg)
        self.count += 1
        self.get_logger().info('Publishing: "%s"' % msg.data, throttle_duration_sec = 10)
        self.get_logger().info('Publish Count: "%s"' % self.count, throttle_duration_sec = 10)
        


def main(args=None):
    rclpy.init(args=args)

    sin_generator = SinGenerator()

    rclpy.spin(sin_generator)

    sin_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()