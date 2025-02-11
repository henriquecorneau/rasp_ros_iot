import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tello_zune import TelloZune
import time

class TelloSubscriber(Node):

    def __init__(self):
        super().__init__('tello_subscriber')

        self.subscription = self.create_subscription(
            String,
            'pico_publisher',
            self.read_commands,
            10)
        self.subscription

        self.tello = TelloZune()
        self.tello.start_tello()
        self.get_logger().info('Tello conected')
        self.last_command = ''

    
    def listener_callback(self, msg):
        self.read_commands(msg)
    

    def read_commands(self, msg):
        self.get_logger().info('Nothing')
        if msg.data and msg.data != self.last_command:  # Só executa se for diferente do último comando
            if msg.data == 'takeoff':
                self.tello.send_cmd("takeoff")
                time.sleep(4)
                self.get_logger().info('Takeoff')

            elif msg.data == 'land':
                self.tello.send_cmd("land")
                time.sleep(4)
                self.get_logger().info('Land')

            # Atualiza o último comando somente se um comando válido foi enviado
            self.last_command = msg.data 



def main(args=None):
    rclpy.init(args=args)

    tello_subscriber = TelloSubscriber()

    rclpy.spin(tello_subscriber)

    tello_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
