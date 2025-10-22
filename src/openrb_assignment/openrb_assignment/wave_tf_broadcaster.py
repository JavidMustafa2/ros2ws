import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class WaveTFBroadcaster(Node):

    def __init__(self):
        super().__init__('wave_tf_broadcaster')

 
        self.declare_parameter('parent_frame', "map") 
        self.declare_parameter('child_frame', "sin_frame")
        self.declare_parameter('topic', "sinSignal")
        self.declare_parameter('rate_hz',30.0)
        #extracting params
        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.topic = self.get_parameter('topic').value
        self.rate = self.get_parameter('rate_hz').value

        self.subscription = self.create_subscription(
            Float64,
            self.topic,
            self.sin_callback,
            10)
        self.subscription

        self.br = TransformBroadcaster(self)
        self.angle = 0.0

        self.timer = self.create_timer(1/self.rate, self.broadcast)


    def sin_callback(self, msg):
            self.angle = msg.data 
    
    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent          
        t.child_frame_id = self.child   

       
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

    
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.angle*math.pi/180 / 2.0)
        t.transform.rotation.w = math.cos(self.angle*math.pi/180 / 2.0)

        self.br.sendTransform(t)
    
def main(args=None):
    rclpy.init(args=args)
    wave_tf_broadcaster = WaveTFBroadcaster()
    rclpy.spin(wave_tf_broadcaster)
    wave_tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()