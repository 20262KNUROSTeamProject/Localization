import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from goalsetting_interfaces.action import GoToBuilding


class GoalsettingClient(Node):

    def __init__(self):
        super().__init__('goalsetting_client')

        self.action_client = ActionClient(
            self,
            GoToBuilding,
            'go_to_building'
        )

    def send_goal(self, building_name):
        self.get_logger().info(f'서버 연결 대기 중...')

        self.action_client.wait_for_server()

        goal_msg = GoToBuilding.Goal()
        goal_msg.building_name = building_name

        self.get_logger().info(f'목적지 요청 전송: {building_name}')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn(
                '등록되지 않은 건물입니다. 건물 이름을 다시 확인해주세요.'
            )
            building_name = input(
                '이동할 건물 이름을 다시 입력하세요: '
            ).strip()
            
            self.send_goal(building_name)
            return
        
        self.get_logger().info('목적지 요청이 승인되었습니다.')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'{feedback.status} (진행률 {feedback.progress:.0f}%)'
        )

    def result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().warn(result.message)

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    building_name = input('이동할 건물 이름을 입력하세요: ').strip()

    node = GoalsettingClient()
    node.send_goal(building_name)

    rclpy.spin(node)

    node.destroy_node()


if __name__ == '__main__':
    main()
