import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PathBuilder(Node):
    def __init__(self):
        super().__init__('path_builder')

        # Substitua pelo tópico que vem do Isaac ROS
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',  # ou o tópico real do seu SLAM
            self.pose_callback,
            qos_profile_sensor_data
        )

        # Tópico de saída para o RViz
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.path_pub = self.create_publisher(Path, '/drone_path', qos_profile)

        self.path = Path()
        self.counter = 0

        self.get_logger().info("PathBuilder iniciado e escutando /visual_slam/tracking/vo_pose")

    def pose_callback(self, msg):
        # Pega o header do frame original
        self.path.header = msg.header

        # Adiciona pose a cada N mensagens (reduz carga)
        self.counter += 1
        if self.counter % 10 == 0:  # a cada 10 poses
            self.path.poses.append(msg)

            # Mantém no máximo 500 pontos pra não sobrecarregar o RViz
            if len(self.path.poses) > 500:
                self.path.poses.pop(0)

            self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = PathBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
