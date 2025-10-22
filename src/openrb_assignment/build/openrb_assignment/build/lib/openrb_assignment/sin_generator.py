import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from math import sin
from math import pi
from time import time 
from geometry_msgs.msg import Twist
class SinGenerator(Node):

    def __init__(self):
        super().__init__('sin_generator')
        #make pub
        self.publisher_ = self.create_publisher(Float64, 'sinSignal', 10)
        #declaring params
        self.declare_parameter('frequency', 1.0) #hertz
        self.declare_parameter('amplitude', 1.0) 
        self.declare_parameter('offset', 0.0) #degrees
        self.declare_parameter('timer_period',0.1)
        #extracting params
        self.timer_period = self.get_parameter('timer_period').value# seconds
        self.freq= self.get_parameter('frequency').value
        self.amp = self.get_parameter('amplitude').value
        self.off = self.get_parameter('offset').value
        #make params live editable
        self.add_on_set_parameters_callback(self.param_callback)
        #create timer and counter
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.count = 0
        self.mode = 0
        #create keyboard input listener
        self.create_subscription(Twist, "/cmd_vel",self.keyboard_input_callback,10)
    
    def param_callback(self,params):
        for param in params:
            if param.name == "frequency":
                self.freq = param.value
            if param.name == "amplitude":
                self.amp = param.value
            if param.name == "offset":
                self.off = param.value
        return SetParametersResult(successful=True)

    def param_change_select(self,mode,msg):
        if (self.mode == 0) and (msg.linear.x > 0):
            update = self.get_parameter("amplitude").value + 1.0
            self.set_parameters([Parameter("amplitude", Parameter.Type.DOUBLE, value = update)])
        #press m or , or . when using teleop twist keyboard
        elif (self.mode == 0) and (msg.linear.x < 0):
            update = self.get_parameter("amplitude").value - 1.0
            self.set_parameters([Parameter("amplitude", Parameter.Type.DOUBLE, value = update)])
         #press i or u or o when using telop twist keyboard
        if (self.mode == 1) and (msg.linear.x > 0):
            update = self.get_parameter("frequency").value + 1.0
            self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])
        #press m or , or . when using teleop twist keyboard
        elif (self.mode == 1) and (msg.linear.x < 0):
            update = self.get_parameter("frequency").value - 1.0
            self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])
         #press i or u or o when using telop twist keyboard
        if (self.mode == 2) and (msg.linear.x > 0):
            update = self.get_parameter("frequency").value + (pi/4)
            self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])
        #press m or , or . when using teleop twist keyboard
        elif (self.mode == 2) and (msg.linear.x < 0):
            update = self.get_parameter("frequency").value - (pi/4)
            self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])

    def keyboard_input_callback(self,msg: Twist):
       
        def param_change_select(msg: Twist):
            if (self.mode == 0) and (msg.linear.x > 0):
                update = self.get_parameter("amplitude").value + 1.0
                self.set_parameters([Parameter("amplitude", Parameter.Type.DOUBLE, value = update)])
            #press m or , or . when using teleop twist keyboard
            elif (self.mode == 0) and (msg.linear.x < 0):
                update = self.get_parameter("amplitude").value - 1.0
                self.set_parameters([Parameter("amplitude", Parameter.Type.DOUBLE, value = update)])
            #press i or u or o when using telop twist keyboard
            if (self.mode == 1) and (msg.linear.x > 0):
                update = self.get_parameter("frequency").value + 1.0
                self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])
            #press m or , or . when using teleop twist keyboard
            elif (self.mode == 1) and (msg.linear.x < 0):
                update = self.get_parameter("frequency").value - 1.0
                self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])
            #press i or u or o when using telop twist keyboard
            if (self.mode == 2) and (msg.linear.x > 0):
                update = self.get_parameter("frequency").value + (pi/4)
                self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])
            #press m or , or . when using teleop twist keyboard
            elif (self.mode == 2) and (msg.linear.x < 0):
                update = self.get_parameter("frequency").value - (pi/4)
                self.set_parameters([Parameter("frequency", Parameter.Type.DOUBLE, value = update)])

        #press j when using telop twist keyboard
        if msg.angular.z > 0 and self.mode <=1:
            self.mode += 1
        #press l when using teleop twist keyboard
        elif msg.angular.z < 0 and self.mode >=1:
            self.mode -= 1
        else:
            param_change_select(msg)
        
        
 
  

    def timer_callback(self):
        passed_time = (self.get_clock().now() - self.start_time).nanoseconds*1e-9 #seconds
        msg = Float64()
        msg.data = self.amp*sin(2*pi*self.freq*passed_time + (self.off*pi)/180)
        self.publisher_.publish(msg)
        self.count += 1
        # self.get_logger().info('Publishing: "%s"' % self.mode, throttle_duration_sec = 0.01)
        # self.get_logger().info('Publish Count: "%s"' % self.count, throttle_duration_sec = 10)
        


def main(args=None):
    rclpy.init(args=args)

    sin_generator = SinGenerator()

    rclpy.spin(sin_generator)

    sin_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()