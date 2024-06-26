import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
from math import atan2, sqrt, pi, cos, sin
from random import uniform
from std_srvs.srv import Empty


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.turtle_pose = {}
        self.pub_cmd_vel = {}
        self.pub_cmd_vel['turtle1'] = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.client_spawn = self.create_client(Spawn, 'spawn')
        self.client_kill = self.create_client(Kill, 'kill')
        self.client_clear = self.create_client(Empty, 'clear')  # 创建清除轨迹的客户端
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer_spawn = self.create_timer(3.0, self.spawn_turtle)
        self.timer_control = self.create_timer(0.1, self.control_turtles)
        self.timer_clear = self.create_timer(3.0, self.clear_trails)  # 每5秒清除一次轨迹
        self.following_dict = {'turtle1': None}
        self.active_target = 'turtle1'
        self.captured_turtles = []

    def pose_callback(self, msg, turtle_name='turtle1'):
        self.turtle_pose[turtle_name] = msg

    def clear_trails(self):
        if self.client_clear.service_is_ready():
            request = Empty.Request()
            future = self.client_clear.call_async(request)
            future.add_done_callback(self.handle_clear_response)

    def handle_clear_response(self, future):
        try:
            future.result()  # Confirm the result of the service invocation
            self.get_logger().info('Trails cleared.')
        except Exception as e:
            self.get_logger().error('Failed to call clear service: %s' % str(e))

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = uniform(0.5, 10.5)
        request.y = uniform(0.5, 10.5)
        request.theta = uniform(0, 2 * pi)
        request.name = f'new_turtle{len(self.turtle_pose)}'
        future = self.client_spawn.call_async(request)
        future.add_done_callback(self.handle_spawn)

    def handle_spawn(self, future):
        response = future.result()
        if response:
            self.get_logger().info(f'Successfully spawned {response.name}')
            self.create_subscription(Pose, f'/{response.name}/pose',
                                     lambda msg, name=response.name: self.pose_callback(msg, turtle_name=name), 10)
            self.pub_cmd_vel[response.name] = self.create_publisher(Twist, f'/{response.name}/cmd_vel', 10)
            self.following_dict[response.name] = self.active_target  # The new turtle should follow the current activity goal

    def control_turtles(self):
        if 'turtle1' not in self.turtle_pose:
            return

        leader_pose = self.turtle_pose['turtle1']
        for turtle_name in list(self.following_dict.keys()):
            if turtle_name != 'turtle1' and turtle_name not in self.captured_turtles:
                target_pose = self.turtle_pose[turtle_name]
                distance = sqrt((target_pose.x - leader_pose.x) ** 2 + (target_pose.y - leader_pose.y) ** 2)
                if distance < 1.0:
                    self.captured_turtles.append(turtle_name)
                    self.active_target = turtle_name
                else:
                    # Move initial turtle towards the target
                    twist = Twist()
                    twist.linear.x = min(5.0, distance)
                    angle_to_target = atan2(target_pose.y - leader_pose.y, target_pose.x - leader_pose.x)

                    # Calculate the Angle difference and limit the angular velocity
                    max_angular_speed = 4.0  # Maximum angular velocity, can be adjusted
                    angle_difference = angle_to_target - leader_pose.theta
                    # Angle normalization, make sure it's between -π and π
                    if angle_difference > pi:
                        angle_difference -= 2 * pi
                    elif angle_difference < -pi:
                        angle_difference += 2 * pi

                    # Apply an angular velocity limit
                    twist.angular.z = max(-max_angular_speed, min(max_angular_speed, 6.0 * angle_difference))
                    self.pub_cmd_vel['turtle1'].publish(twist)

        for turtle_name in self.captured_turtles:
            leader_name = self.following_dict[turtle_name]
            if leader_name in self.turtle_pose:
                self.follow_leader(turtle_name, self.turtle_pose[leader_name], self.turtle_pose[turtle_name])

    def follow_leader(self, turtle_name, leader_pose, target_pose):
        follow_distance = 0.05
        angle_behind_leader = leader_pose.theta - pi
        desired_x = leader_pose.x + follow_distance * cos(angle_behind_leader)
        desired_y = leader_pose.y + follow_distance * sin(angle_behind_leader)

        twist = Twist()
        distance_to_desired = sqrt((target_pose.x - desired_x) ** 2 + (target_pose.y - desired_y) ** 2)
        twist.linear.x = min(5.0, distance_to_desired)
        angle_to_desired = atan2(desired_y - target_pose.y, desired_x - target_pose.x)

        # Limit changes in angular speed to prevent too fast rotation
        max_angular_speed = 4.0  # This value can be adjusted for best performance
        angle_difference = (angle_to_desired - target_pose.theta)
        if angle_difference > pi:
            angle_difference -= 2 * pi
        elif angle_difference < -pi:
            angle_difference += 2 * pi
        twist.angular.z = max(-max_angular_speed, min(max_angular_speed, 4.0 * angle_difference))

        self.pub_cmd_vel[turtle_name].publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
