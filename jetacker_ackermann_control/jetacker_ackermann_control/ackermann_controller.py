#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class AckermannController(Node):
    """
    /cmd_vel의 선속도와 각속도를 Ackermann 조향 명령으로 변환합니다.

    입력:
      /cmd_vel
        linear.x  : 차량 전후 속도 [m/s]
        angular.z : 차량 yaw 각속도 [rad/s]

    출력:
      /steering_position_controller/commands
        [front_left_steering_angle,
         front_right_steering_angle]

      /rear_wheel_velocity_controller/commands
        [rear_left_wheel_velocity,
         rear_right_wheel_velocity]
    """

    def __init__(self):
        super().__init__('ackermann_controller')

        # JetAcker 기하학 파라미터
        self.declare_parameter('wheelbase', 0.213)
        self.declare_parameter('track_width', 0.183)
        self.declare_parameter('wheel_radius', 0.05)

        # 제한값
        self.declare_parameter('max_steering_angle', 0.70)
        self.declare_parameter('max_wheel_velocity', 20.0)
        self.declare_parameter('command_timeout', 0.5)

        self.wheelbase = float(
            self.get_parameter('wheelbase').value
        )
        self.track_width = float(
            self.get_parameter('track_width').value
        )
        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value
        )
        self.max_wheel_velocity = float(
            self.get_parameter('max_wheel_velocity').value
        )
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        self.last_command_time = self.get_clock().now()
        self.timeout_warning_printed = False

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.steering_publisher = self.create_publisher(
            Float64MultiArray,
            '/steering_position_controller/commands',
            10
        )

        self.rear_wheel_publisher = self.create_publisher(
            Float64MultiArray,
            '/rear_wheel_velocity_controller/commands',
            10
        )

        # 명령이 끊겼을 때 자동 정지
        self.timeout_timer = self.create_timer(
            0.1,
            self.command_timeout_callback
        )

        self.get_logger().info(
            'Ackermann controller started\n'
            f'  wheelbase: {self.wheelbase:.3f} m\n'
            f'  track width: {self.track_width:.3f} m\n'
            f'  wheel radius: {self.wheel_radius:.3f} m\n'
            '  input: /cmd_vel\n'
            '  steering output: '
            '/steering_position_controller/commands\n'
            '  rear wheel output: '
            '/rear_wheel_velocity_controller/commands'
        )

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(value, maximum))

    def cmd_vel_callback(self, msg: Twist) -> None:
        linear_velocity = float(msg.linear.x)
        angular_velocity = float(msg.angular.z)

        self.last_command_time = self.get_clock().now()
        self.timeout_warning_printed = False

        (
            front_left_angle,
            front_right_angle,
            rear_left_velocity,
            rear_right_velocity
        ) = self.calculate_ackermann_commands(
            linear_velocity,
            angular_velocity
        )

        self.publish_commands(
            front_left_angle,
            front_right_angle,
            rear_left_velocity,
            rear_right_velocity
        )

    def calculate_ackermann_commands(
        self,
        linear_velocity: float,
        angular_velocity: float
    ) -> tuple[float, float, float, float]:

        velocity_epsilon = 1.0e-4
        angular_epsilon = 1.0e-4

        # 정지
        if (
            abs(linear_velocity) < velocity_epsilon
            and abs(angular_velocity) < angular_epsilon
        ):
            return 0.0, 0.0, 0.0, 0.0

        # Ackermann 차량은 제자리 회전 불가
        if abs(linear_velocity) < velocity_epsilon:
            self.get_logger().warning(
                'Ackermann 차량은 제자리 회전을 할 수 없습니다. '
                'linear.x가 0인 angular.z 명령을 무시합니다.'
            )
            return 0.0, 0.0, 0.0, 0.0

        # 직진 또는 후진
        if abs(angular_velocity) < angular_epsilon:
            rear_wheel_velocity = (
                linear_velocity / self.wheel_radius
            )

            rear_wheel_velocity = self.clamp(
                rear_wheel_velocity,
                -self.max_wheel_velocity,
                self.max_wheel_velocity
            )

            return (
                0.0,
                0.0,
                rear_wheel_velocity,
                rear_wheel_velocity
            )

        # 차량 중심 기준 회전 반경
        turning_radius = (
            linear_velocity / angular_velocity
        )

        # 좌우 전륜의 Ackermann 조향각
        left_denominator = (
            turning_radius - self.track_width / 2.0
        )
        right_denominator = (
            turning_radius + self.track_width / 2.0
        )

        front_left_angle = math.atan2(
            self.wheelbase,
            left_denominator
        )

        front_right_angle = math.atan2(
            self.wheelbase,
            right_denominator
        )

        # atan2 결과가 불필요하게 ±pi 근처로 가지 않도록 보정
        front_left_angle = math.atan(
            math.tan(front_left_angle)
        )
        front_right_angle = math.atan(
            math.tan(front_right_angle)
        )

        front_left_angle = self.clamp(
            front_left_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        front_right_angle = self.clamp(
            front_right_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        # 후륜 좌우의 실제 선속도
        rear_left_linear_velocity = (
            linear_velocity
            - angular_velocity * self.track_width / 2.0
        )

        rear_right_linear_velocity = (
            linear_velocity
            + angular_velocity * self.track_width / 2.0
        )

        # 선속도 [m/s] → 바퀴 각속도 [rad/s]
        rear_left_wheel_velocity = (
            rear_left_linear_velocity / self.wheel_radius
        )

        rear_right_wheel_velocity = (
            rear_right_linear_velocity / self.wheel_radius
        )

        rear_left_wheel_velocity = self.clamp(
            rear_left_wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        rear_right_wheel_velocity = self.clamp(
            rear_right_wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        return (
            front_left_angle,
            front_right_angle,
            rear_left_wheel_velocity,
            rear_right_wheel_velocity
        )

    def publish_commands(
        self,
        front_left_angle: float,
        front_right_angle: float,
        rear_left_velocity: float,
        rear_right_velocity: float
    ) -> None:

        steering_command = Float64MultiArray()
        steering_command.data = [
            front_left_angle,
            front_right_angle
        ]

        rear_wheel_command = Float64MultiArray()
        rear_wheel_command.data = [
            rear_left_velocity,
            rear_right_velocity
        ]

        self.steering_publisher.publish(steering_command)
        self.rear_wheel_publisher.publish(rear_wheel_command)

        self.get_logger().info(
            'Ackermann command | '
            f'steer L={front_left_angle:+.3f}, '
            f'R={front_right_angle:+.3f} rad | '
            f'rear wheel L={rear_left_velocity:+.3f}, '
            f'R={rear_right_velocity:+.3f} rad/s'
        )

    def command_timeout_callback(self) -> None:
        elapsed_time = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1.0e9

        if elapsed_time > self.command_timeout:
            self.publish_commands(
                0.0,
                0.0,
                0.0,
                0.0
            )

            if not self.timeout_warning_printed:
                self.get_logger().warning(
                    'cmd_vel timeout: 차량을 정지합니다.'
                )
                self.timeout_warning_printed = True


def main(args=None):
    rclpy.init(args=args)

    node = AckermannController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_commands(
            0.0,
            0.0,
            0.0,
            0.0
        )
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
