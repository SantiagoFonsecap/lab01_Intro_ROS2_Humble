import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios
from turtlesim.srv import TeleportAbsolute
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Control con teclado iniciado")

    def draw_M(self):
        msg = Twist()
        msg.linear.x = 4.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 0.0
        msg.angular.z = -2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.angular.z = 0.0
        msg.linear.x = 2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 0.0
        msg.angular.z = -2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.angular.z = 0.0
        msg.linear.x = -2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 0.0
        msg.angular.z = 4.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.angular.z = 0.0
        msg.linear.x = -4.0
        self.publisher_.publish(msg)
        time.sleep(1.0)

    def draw_F(self):
        msg = Twist()
        msg.linear.x = 4.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 0.0
        msg.angular.z = -1.5
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = -2.0
        self.publisher_.publish(msg)#Regresa de primer linea horizontal
        time.sleep(1.0)
        msg.linear.x = 0.0
        msg.angular.z = 1.5
        self.publisher_.publish(msg)
        time.sleep(1.0)#Regresa al angulo inicial
        msg.angular.z = 0.0
        msg.linear.x = -2.0
        self.publisher_.publish(msg)#Se ubica en el centro 
        time.sleep(1.0)
        msg.linear.x = 0.0
        msg.angular.z = -1.5
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(1.0)       

    def draw_C(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        self.publisher_.publish(msg)
        time.sleep(1.0)

    def draw_S(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        self.publisher_.publish(msg)
        time.sleep(1.0)
        msg.angular.z = -2.0
        self.publisher_.publish(msg)
        time.sleep(1.0)
        self.publisher_.publish(msg)
        time.sleep(1.0)

    def draw_J(self):
        msg=Twist()
        msg.linear.x=2.0
        self.publisher_.publish(msg)
        time.sleep(2.0)
        msg.linear.x = 0.0
        msg.angular.z = 2.0
        self.publisher_.publish(msg)
        time.sleep(2.0)
        msg.angular.z = 0.0
        msg.linear.x = 3.0
        self.publisher_.publish(msg)
        time.sleep(2.0)
        msg.linear.x = 0.0
        msg.angular.z = 2.0
        self.publisher_.publish(msg)
        time.sleep(2.0)
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(2.0)
        msg.linear.x = -2.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(2.0)

    def draw_D(self):
        msg=Twist()
        msg.linear.x = 4.0
        msg.angular.z = -3.8
        self.publisher_.publish(msg)
        time.sleep(2.0)   
        msg.linear.x = 0.0
        msg.angular.z = -1.8
        self.publisher_.publish(msg)
        time.sleep(2.0) 
        msg.linear.x = 2.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(2.0)

    def draw_P(self):
        msg=Twist()
        msg.linear.x = 4.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(2.0)
        msg.linear.x = 0.0
        msg.angular.z = -1.8
        self.publisher_.publish(msg)
        time.sleep(2.0) 
        msg.linear.x = 2.0
        msg.angular.z = -3.8
        self.publisher_.publish(msg)
        time.sleep(2.0)   
    
    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def move_turtle(self):
        while rclpy.ok():
            key = self.get_key()
            msg = Twist()
            if key == '\x1b':  # ESC sequence
                if sys.stdin.read(1) == '[':
                    arrow = sys.stdin.read(1)
                    if arrow == 'A':  # Flecha ↑
                        msg.linear.x = 2.0
                    elif arrow == 'B':  # Flecha ↓
                        msg.linear.x = -2.0
                    elif arrow == 'C':  # Flecha →
                        msg.angular.z = -2.0
                    elif arrow == 'D':  # Flecha ←
                        msg.angular.z = 2.0
                    self.publisher_.publish(msg)    
            elif key.lower() == 'm':
                self.get_logger().info("Dibujando M...")
                self.draw_M()
            elif key.lower() == 'f':
                self.get_logger().info("Dibujando F...")
                self.draw_F()
            elif key.lower() == 'c':
                self.get_logger().info("Dibujando C...")
                self.draw_C()
            elif key.lower() == 's':
                self.get_logger().info("Dibujando S...")
                self.draw_S()
            elif key.lower() == 'j':
                self.get_logger().info("Dibujando j...")
                self.draw_J()
            elif key.lower() == 'd':
                self.get_logger().info("Dibujando D...")
                self.draw_D()
            elif key.lower() == 'p':
                self.get_logger().info("Dibujando P...")
                self.draw_P()
            

def main(args=None):
    rclpy.init(args=args)
    move_turtle = TurtleController()
    try:
        move_turtle.move_turtle()
    except KeyboardInterrupt:
        pass
    TurtleController.destroy_node()
    rclpy.shutdown()
