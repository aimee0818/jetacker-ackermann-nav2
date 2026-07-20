#!/usr/bin/env python3

import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigateToPoseCLI(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_cli')

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """Yaw 각도(rad)를 z, w 쿼터니언으로 변환합니다."""
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        self.get_logger().info(
            '/navigate_to_pose 액션 서버를 기다리는 중입니다...'
        )

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'NavigateToPose 액션 서버를 찾을 수 없습니다. '
                'Nav2가 실행 중인지 확인하세요.'
            )
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        quaternion_z, quaternion_w = self.yaw_to_quaternion(yaw)

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = quaternion_z
        goal_msg.pose.pose.orientation.w = quaternion_w

        self.get_logger().info(
            f'목표 전송: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표가 승인되었습니다.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error('결과를 받지 못했습니다.')
            return

        status = result.status

        if status == 4:
            self.get_logger().info('목적지에 도착했습니다.')
        elif status == 5:
            self.get_logger().warning('목표가 취소되었습니다.')
        elif status == 6:
            self.get_logger().error('목표 이동에 실패했습니다.')
        else:
            self.get_logger().warning(
                f'이동이 종료되었습니다. status={status}'
            )

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining

        self.get_logger().info(
            f'남은 거리: {distance_remaining:.2f} m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseCLI()

    try:
        print('\n========================================')
        print(' JetAcker NavigateToPose CLI')
        print(' yaw 입력 단위: degree')
        print('========================================')

        x = float(input('목적지 x [m]: '))
        y = float(input('목적지 y [m]: '))
        yaw_degree = float(input('목적지 yaw [degree]: '))

        yaw_radian = math.radians(yaw_degree)

        node.send_goal(
            x=x,
            y=y,
            yaw=yaw_radian
        )

    except ValueError:
        node.get_logger().error('숫자를 정확히 입력해 주세요.')

    except KeyboardInterrupt:
        node.get_logger().info('사용자가 종료했습니다.')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
