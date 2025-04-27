import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios
import time
from turtlesim.msg import Pose
import threading
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) #Publisher para enviar velocidades 
        self.theta=0.0
        self.lock = threading.Lock()
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.actualizar_pose, 10) #Para actualizar la posición de la tortuga
        """self.pose = None"""
        self.running = True

        self.input_thread = threading.Thread(target=self.move_turtle) #Crear hilo para recibir en paralelo los comandos por teclado
        self.input_thread.daemon = True
        self.input_thread.start()

    def actualizar_pose(self, msg):  #Actualización de la orientación de la tortuga
        self.theta = msg.theta
        with self.lock:
            self.theta = msg.theta

    def alinear_tortuga(self, angulo_objetivo): #Similar a control, es una función que busca reducir el error ante un ángulo objetivo, aunque tiene una velocidad fija.
        tolerancia = 0.007
        max_duracion = 15
        velocidad = 0.2

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < max_duracion:
            with self.lock:
                error = self._normalizar_angulo(angulo_objetivo - self.theta) #Sirve para encontrar el camino más cerca al ángulo objetivo (-pi, pi)

            if abs(error) < tolerancia: #Cuando el error está en el rango de tolerancia, deja de actualizarse
                break

            msg = Twist()
            msg.angular.z = velocidad if error > 0 else -velocidad
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.detener()

    def _normalizar_angulo(self, angulo): #Sirve para encontrar el camino más cerca al ángulo objetivo (-pi, pi)
        return math.atan2(math.sin(angulo), math.cos(angulo))
    
    def mover(self, lin_x, ang_z, duracion): # Función que permite movimientos rectos, curvos y giros sobre propio eje según parámetros
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        tiempo_inicial = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - tiempo_inicial < duracion: #Para tener un control mas preciso de la duración del movimiento
            self.publisher_.publish(msg)
            time.sleep(0.1)

    def detener(self): #similar a la función mover pero con los parametros en 0 para evitar movimientos rectos o angulares
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def draw_J(self):
        self.alinear_tortuga(0.0) #Apuntar hacia derecha
        self.mover(1.0, 0.0, 1.0) #Parte superior
        self.alinear_tortuga(-math.pi / 2)  # Apuntar hacia abajo
        self.mover(1.5, 0.0, 2)      # Hacia abajo, linea vertical
        self.mover(1.0, -1.5, 1.5)   # Curva
        self.detener()    

    def draw_D(self):
        self.alinear_tortuga(math.pi / 2) #Apuntar arriba
        self.mover(3.0, 0.0, 1.0) #Parte derecha
        self.alinear_tortuga(0.0) #Apuntar hacia derecha
        self.mover(1.0, -1.0, 2.2) #Barriga

    def draw_M(self):
        self.alinear_tortuga(math.pi / 2)  # Apuntar hacia arriba
        self.mover(1.5, 0.0, 2.0) #Linea vertical izquierda
        self.alinear_tortuga(-math.pi / 4) # Apuntar a -45deg
        self.mover(1.5, 0.0, 1.0) #Linea diagonal izquierda
        self.alinear_tortuga(math.pi / 4) # Apuntar a 45deg
        self.mover(1.5, 0.0, 1.0) #Linea diagonal derecha
        self.alinear_tortuga(-math.pi / 2) # Apuntar a 45deg
        self.mover(1.5, 0.0, 2.0) #Linea diagonal derecha

    def draw_S(self):
        self.alinear_tortuga(-3 * math.pi / 4) $ 
        self.mover(1.0, 1.5, 3.0) 
        self.mover(1.0, -1.5, 3.0)
        
    def draw_C(self):
        self.alinear_tortuga(-math.pi / 2) #Apuntar abajo
        self.mover(1.0, -1.5, 1.8) #Curva inferior
        self.mover(2.0, 0.0, 1.0) #Recta
        self.mover(1.0, -1.5, 1.8) #Curva superior
    
    def draw_F(self):
        self.alinear_tortuga(math.pi / 2) #Apuntar arriba
        self.mover(2.0, 0.0, 2.0) #Vertical
        self.alinear_tortuga(0.0) #Apuntar hacia derecha
        self.mover(2.0, 0.0, 1.0) #Horizontal superior
        self.mover(-2.0, 0.0, 1.0)
        self.alinear_tortuga(math.pi / 2) #Apuntar arriba
        self.mover(-2.0, 0.0, 1.0) #Reroceder a la mitad
        self.alinear_tortuga(0.0) #Apuntar hacia derecha
        self.mover(1.5, 0.0, 1.0) #Horizontal superior

    def draw_P(self):
        self.alinear_tortuga(math.pi / 2) #Apuntar arriba
        self.mover(2.0, 0.0, 2.0) #Parte derecha (Recta)
        self.alinear_tortuga(0.0) #Apuntar hacia derecha
        self.mover(1.0, -1.5, 1.5) #Barriga
    
    def get_key(self):  #Función que obtiene el comando de teclado
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b': #Se revisa si corresponde a una flecha
                ch2 = sys.stdin.read(1) #Se identifica cual de todas
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3
            else:
                return ch1 #Si no es una flecha, pasa la letra
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def move_turtle(self):
        print("Controles:") #Consola donde se identifican las funciones que se pueden usar
        print("  j - Dibujar letra J ")
        print("  d - Dibujar letra D ")
        print("  m - Dibujar letra M ")
        print("  p - Dibujar letra P ")
        print("  s- Dibujar letra S ")
        print("  c- Dibujar letra C ")
        print("  f- Dibujar letra F ")
        print("  Flechas - Mover la tortuga (↑ ↓ ← →)")
        while self.running: #Dependiendo de la letra de entrada accederá a la función correspondiente para que sea dibujada por la tortuga
            key = self.get_key()
            if key == '\x1b[A':
                self.mover(1.5, 0.0, 0.2)  
            elif key == '\x1b[B':
                self.mover(-1.5, 0.0, 0.2)    
            elif key == '\x1b[C':
                self.mover(0.0, -1.0, 0.2)
            elif key == '\x1b[D':
                self.mover(0.0, 1.0, 0.2)
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
                self.get_logger().info("Dibujando J...")
                self.draw_J()
            elif key.lower() == 'd':
                self.get_logger().info("Dibujando D...")
                self.draw_D()
            elif key.lower() == 'p':
                self.get_logger().info("Dibujando P...")
                self.draw_P()
            

def main(args=None):
    rclpy.init(args=args) #Inicializa ROS2
    node = TurtleController() #Crea la instancia de TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
