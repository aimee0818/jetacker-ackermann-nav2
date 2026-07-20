#!/usr/bin/env python3

import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigateToPlace(Node):
    def __init__(self):
        super().__init__('navigate_to_place')

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # 장소별 목표 좌표
        # 형식: "장소 이름": (x, y, yaw_degree)
                # 장소별 목표 좌표
        # 형식: '장소': (x, y, yaw_degree)
        self.locations = {
            'sofa': (
                -0.9539327621459961,
                -2.860058307647705,
                0.0
            ),
            'bed': (
                2.478656768798828,
                -3.0,
                0.0
            ),
            'kitchen': (
                0.5685291290283203,
                1.819406270980835,
                0.0
            ),
            'home': (
                -4.822980880737305,
                -0.4989560842514038,
                0.0
            ),
            'table': (
                -3.4194962978363037,
                2.9903957843780518,
                0.0
            ),
        }

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def print_locations(self) -> None:
        print('\n등록된 장소')
        print('------------------------------')

        for name, (x, y, yaw_degree) in self.locations.items():
            print(
                f'{name:10s} '
                f'x={x:+.2f}, y={y:+.2f}, yaw={yaw_degree:+.1f}°'
            )

        print('------------------------------')

    def send_goal(self, place_name: str) -> None:
        place_name = place_name.strip().lower()

        if place_name not in self.locations:
            self.get_logger().error(
                f'등록되지 않은 장소입니다: {place_name}'
            )
            self.print_locations()
            return

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'NavigateToPose 액션 서버를 찾을 수 없습니다. '
                'Nav2가 실행 중인지 확인하세요.'
            )
            return

        x, y, yaw_degree = self.locations[place_name]
        yaw_radian = math.radians(yaw_degree)

        quaternion_z, quaternion_w = self.yaw_to_quaternion(
            yaw_radian
        )

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = quaternion_z
        goal_msg.pose.pose.orientation.w = quaternion_w

        self.get_logger().info(
            f'장소 이동 요청: {place_name} | '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw_degree:.1f}°'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표가 승인되었습니다.')

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error('결과를 받지 못했습니다.')
            return

        status = result.status

        if status == 4:
            self.get_logger().info(
                f'{place_name}에 도착했습니다.'
            )
        elif status == 5:
            self.get_logger().warning('이동이 취소되었습니다.')
        elif status == 6:
            self.get_logger().error('이동에 실패했습니다.')
        else:
            self.get_logger().warning(
                f'이동 종료 status={status}'
            )

    def feedback_callback(self, feedback_msg) -> None:
        distance_remaining = (
            feedback_msg.feedback.distance_remaining
        )

        self.get_logger().info(
            f'남은 거리: {distance_remaining:.2f} m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)

    node = NavigateToPlace()

    try:
        print('\n========================================')
        print(' JetAcker Place Navigation')
        print('========================================')

        node.print_locations()

        place_name = input(
            '이동할 장소 이름을 입력하세요: '
        )

        node.send_goal(place_name)

    except KeyboardInterrupt:
        node.get_logger().info('사용자가 종료했습니다.')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
