import csv
import rclpy

from rclpy.node import Node

from turtlesim.msg import Pose as TPose

from geometry_msgs.msg import Twist
from collections import deque

MAX_DIFF = 0.1


class MissionControl(deque):
    
    # Define a sequência de pontos para cada formato disponível
    pontos = {
        
    }
    
    def __init__(self, csv_file="pontos.csv"):
        super().__init__()
        with open(csv_file) as csv_file_var:
            csv_reader = csv.reader(csv_file_var, delimiter=',')
            for row in csv_reader:
        #         # Enfileirando posições
                new_pose = Pose()
                new_pose.x, new_pose.y = [float(x) for x in row]
                self.enqueue(new_pose)
                # Empilhando posiçoes
        #     for row in csv_reader:
        # #         # Enfileirando posições
        #         new_pose = Pose()
        #         new_pose.x, new_pose.y = [float(x) for x in row]
        #         self.enqueue(new_pose)


        


    def enqueue(self, x):
        super().append(x)

    def stack(self, x):
        super().appendleft(x)

    def dequeue(self, setpoint):
        # super().append(setpoint)
        return super().popleft()
    

class Pose(TPose):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)

    # retorna quando a classe é printada
    def __repr__(self):
        return f'(x={self.x}, y={self.y})'
    
    # retorna quando 2 poses(classe) são adicionadas '+'
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    # retorna quando 2 poses(classe) são subtraidas '-'
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
        
    # Overload da comparação entre posições '=='
    def __eq__(self, other):
        return abs(self.x - other.x) < MAX_DIFF and abs(self.y - other.y) < MAX_DIFF

class TurtleController(Node):
    def __init__(self, mission_control, control_period=0.02):
        super().__init__('turtlecontroller')
        # pose inicial
        self.pose = Pose(x=-40.0)
        # instanciando definidor de posição
        self.setpoint = Pose(x=-40.0)

        # Instanciando lista de posições já percorridas


        # Mission control
        self.mission_control = mission_control
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )
        # subscrição
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic='/turtle1/pose',
            callback=self.pose_callback,
            qos_profile=10
        )
        self.control_timer = self.create_timer(
            timer_period_sec=control_period,
            callback=self.control_callback
        )

    def control_callback(self):
        if self.pose.x == -40.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        msg = Twist()
        # var armazena a diff da pos atual e da setada
        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 1.0 if x_diff > 0 else -1.0
        else:
            msg.linear.x = 0.0
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 1.0 if y_diff > 0 else -1.0
        else:
            msg.linear.y = 0.0

        self.publisher.publish(msg)

    def update_setpoint(self):
        try:
            self.get_logger().info(f"Cheguei em {self.pose}, vou para {self.setpoint}")
            self.mission_control.enqueue(self.setpoint)
            self.setpoint = self.pose + self.mission_control.dequeue(self.setpoint)
        except IndexError:
            self.get_logger().info("O trajeto finalizou")
            exit()


    def pose_callback(self, msg):
        self.pose = Pose(x=msg.x,  y=msg.y, theta=msg.theta)
        if self.setpoint.x == -40.0:
            self.update_setpoint()
        self.get_logger().info(f'A tartaruga está em x={msg.x}, y={msg.y}, theta={msg.theta}')


def main(args=None):
    rclpy.init(args=args)
    mc = MissionControl()
    tc = TurtleController(mc)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()