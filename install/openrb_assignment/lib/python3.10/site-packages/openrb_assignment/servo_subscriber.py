import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import Float64
from threading import RLock
from openrb_assignment.dynamixel_client import DynamixelClient
import openrb_assignment.dynamixel_registers
import openrb_assignment.utils
import numpy as np
from math import pi

class ServoSubscriber(Node):

    def __init__(self):
        super().__init__('servo_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            'sinSignal',
            self.move_servo_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.declare_parameter('port', '/dev/ttyACM0') #hertz
        self.declare_parameter('baud', 57600) 
        self.declare_parameter('id', 1) #degrees
       
        #extracting params
        
        self.port= self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value 
        self.id = self.get_parameter('id').get_parameter_value().integer_value 
        self.cli = DynamixelClient(
                    motor_ids = [self.id],
                    port = self.port ,
                    baudrate = self.baud,
                    lazy_connect=True
        )
        self._motor_lock: Rlock = RLock()

    def enable_torque(self, motor_ids = None):
        motor_ids = [self.id]

        with self._motor_lock:
            self.cli.set_torque_enabled(motor_ids,True, retries= 5)
            self.initial_pos = self.cli.read_pos()
        
      

            
            

    def disable_torque(self, motor_ids = None):
        motor_ids = [self.id]

        with self._motor_lock:
            self.cli.set_torque_enabled(motor_ids,False,retries= 5)
    
 
    def disconnect(self):
        
        with self._motor_lock:
            self.cli.disconnect()
    
    def set_control_mode(self,mode=5,motor_ids=None):
        motor_ids = [self.id]
        with self._motor_lock:
            self.cli.set_operating_mode(motor_ids,mode)
    
    def move_servo_callback(self,msg,motor_ids = None):
        motor_ids = [self.id]
        command = np.array([msg.data*pi/180])
        self.cli.write_desired_pos(motor_ids,self.initial_pos + command)

    
        


def main(args=None):
    rclpy.init(args=args)
    
    servo_subscriber = ServoSubscriber()
    servo_subscriber.enable_torque()
    servo_subscriber.set_control_mode()

    rclpy.spin(servo_subscriber)

    servo_subscriber.disable_torque()
    servo_subscriber.disconnect()
    servo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()