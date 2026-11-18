
import asyncio
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from goalsetting_interfaces.action import GoToBuilding


class GoalsettingServer(Node):

    def __init__(self):
        super().__init__('goalsetting_server')

        self.building_positions = {
            '테크노 빌딩': (1.9018, 4.1184, 0.0),
            '종합 정보 센터': (6.0569, 2.5835, 3.09),
            '도서관': (5.1261, -1.7043, 1.590),
        }

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.action_server = ActionServer(
            self,
            GoToBuilding,
            'go_to_building',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('건물 안내 서버가 실행되었습니다.')
        
    def normalize_building_name(self, name):
        return ''.join(name.split())

    def goal_callback(self, goal_request):
        building = self.normalize_building_name(
            goal_request.building_name
        )
        
        normalized_positions = {
            self.normalize_building_name(name): name
            for name in self.building_positions
        }
        
        if building not in normalized_positions:
            self.get_logger().warn(
                f'등록되지 않은 건물: {goal_request.building_name}'
            )
            return GoalResponse.REJECT
        
        self.get_logger().info(
            f'목적지 요청 승인: {normalized_positions[building]}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('이동 취소 요청을 받았습니다.')
        return CancelResponse.ACCEPT

    def nav_feedback_callback(self, goal_handle, initial_distance, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining

        if initial_distance[0] is None:
            initial_distance[0] = distance

        if initial_distance[0] > 0:
            progress = (1.0 - distance / initial_distance[0]) * 100.0
        else:
            progress = 0.0

        progress = max(0.0, min(100.0, progress))

        feedback = GoToBuilding.Feedback()
        feedback.status = f'이동 중: 남은 거리 {distance:.2f} m'
        feedback.progress = float(progress)

        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f'{feedback.status} ({progress:.1f}%)'
        )

    async def execute_callback(self, goal_handle):
        requested_name = goal_handle.request.building_name
        normalized_name = self.normalize_building_name(requested_name)

        normalized_positions = {
            self.normalize_building_name(name): (name, pose)
            for name, pose in self.building_positions.items()
        }
        
        building, (x, y, yaw) = normalized_positions[normalized_name]

        self.get_logger().info(
            f'Target: {building}, x={x}, y={y}, yaw={yaw}'
        )

        result = GoToBuilding.Result()

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = '이동이 취소되었습니다'
                return result

            self.get_logger().info(
                'Nav2 서버 연결 대기 중...'
            )

        nav_goal = NavigateToPose.Goal()

        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

        nav_goal.pose.pose.position.x = x
        nav_goal.pose.pose.position.y = y
        nav_goal.pose.pose.position.z = 0.0

        nav_goal.pose.pose.orientation.x = 0.0
        nav_goal.pose.pose.orientation.y = 0.0
        nav_goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        nav_goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        initial_distance = [None]

        nav_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda msg: self.nav_feedback_callback(
                goal_handle, initial_distance, msg
            )
        )

        nav_goal_handle = await nav_goal_future

        if not nav_goal_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = 'Nav2 목적지 요청을 거부했습니다.'
            return result

        self.get_logger().info('Nav2 목적지 요청 승인')

        result_future = nav_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                result.message = '이동이 취소되었습니다.'
                return result

            await asyncio.sleep(0.1)

        nav_result = await result_future

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            result.success = True
            result.message = f'{building}에 도착했습니다.'
            self.get_logger().info(result.message)

        else:
            goal_handle.abort()
            result.success = False
            result.message = f'{building} 이동에 실패했습니다.'
            self.get_logger().warn(result.message)

        return result


def main(args=None):
    rclpy.init(args=args)

    node = GoalsettingServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
