import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from dynamixel_client import DynamixelClient
import dynamixel_registers
import utils

class ServoSubscriber(Node):

    def __init__(self):
        super().__init__('servo_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'sinSignal',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.declare_parameter('port', 'ttyACM0') #hertz
        self.declare_parameter('baud', 57600) 
        self.declare_parameter('id', 1) #degrees
       
        #extracting params
        
        self.port= self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().int_value 
        self.id = self.get_parameter('id').get_parameter_value().int_value 
        self.cli = DynamixelClient(
                    motor_ids = [self.id],
                    port = self.port ,
                    baudrate = self.baud,
                    lazy_connect=True
        )

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data, throttle_duration_sec = 10)


def main(args=None):
    rclpy.init(args=args)

    servo_subscriber = ServoSubscriber()

    rclpy.spin(servo_subscriber)

    servo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()