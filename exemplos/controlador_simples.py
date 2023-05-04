#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose

MAX_DIFF = 0.1


class Pose(TPose):
    """
    Essa classe serve só para que eu possa modificar alguns
    dos comportamentos default da classe Pose do turtlesim.
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):
        """Modifica a forma como aparece a classe no `print`"""
        return f"(x={self.x}, theta={self.theta})"
    
    def __eq__(self, other):
        """
        Modifica a funcionalidade do operador `==`
        Passa a considerar como iguais duas poses que 
        estiverem dentro de um range delimitado por `MAX_DIFF`.
        A comparação considera x e theta ao mesmo tempo.
        """
        return abs(self.x - other.x) < MAX_DIFF \
        and abs(self.theta - other.theta) < MAX_DIFF


class TurtleController(Node):
    """
    Classe do controlador. É um nó de ROS
    """
    
    def __init__(self, control_period=0.02):
        """
        Construtor da classe controladora.
        Aqui cria-se o publisher, a subscription e o timer.
        Como parâmetro é possível passar o período do controlador.
        """
        super().__init__('turtle_controller')
        # Aproveitei a classe Pose para meus parâmetros pose e setpoint
        # Iniciei ambos em x=-40.0 pois é um valor impossível no turtlesim
        self.pose = Pose(x=-40.0)
        self.setpoint = Pose(x=-40.0)
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10
        )
        # O timer passa a ser um timer do loop de controle.
        # Não se deve iniciar loop de controle sem definir um período.
        self.control_timer = self.create_timer(
                timer_period_sec=control_period,
                callback=self.control_callback
        )

    def control_callback(self):
        """
        Loop de controle. 
        Aqui é onde define-se o acionamento dos motores do robô.
        Roda a cada `control_period` segundos.
        """
        # Se a pose estiver em -40.0, significa que não chegou nenhuma info ainda
        if self.pose.x == -40.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        msg = Twist()
        theta_diff = self.setpoint.theta - self.pose.theta
        x_diff = self.setpoint.x - self.pose.x
        if self.pose == self.setpoint:
            self.get_logger().info(f"Mbappé chegou em {self.setpoint}")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        if abs(theta_diff) > MAX_DIFF:
            msg.angular.z = 0.5 if theta_diff > 0 else -0.5
            msg.linear.x = 0.0
        elif abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.5 if x_diff > 0 else -0.5
        self.publisher.publish(msg)
        
    def pose_callback(self, msg):
        if self.pose.x == -40.0:
            self.setpoint.x = msg.x + 5.0
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
