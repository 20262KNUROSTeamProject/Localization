
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from goalsetting_interfaces.action import GoToBuilding
from goalsetting_interfaces.srv import CalculateFare
from goalsetting_interfaces.srv import VerifyPickup


class GoalsettingClient(Node):

    def __init__(self):
        super().__init__('goalsetting_client')

        self.action_client = ActionClient(
            self,
            GoToBuilding,
            'go_to_building'
        )

        self.fare_client = self.create_client(
            CalculateFare,
            'calculate_fare'
        )

        self.pickup_client = self.create_client(
            VerifyPickup,
            'verify_pickup'
        )

    def wait_for_future(self, future, description):
        """ROS executor가 처리하는 Future의 완료를 기다린다."""
        self.get_logger().info(f'{description} 응답 대기 중...')

        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if not rclpy.ok():
            return None

        try:
            return future.result()
        except Exception as error:
            self.get_logger().error(
                f'{description} 처리 중 오류: {error}'
            )
            return None

    def wait_for_service(self, client, service_name):
        while rclpy.ok() and not client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                f'{service_name} 서비스 연결 대기 중...'
            )

        return rclpy.ok()

    def request_fare(self, building_name, luggage_weight):
        if not self.wait_for_service(
            self.fare_client,
            'calculate_fare'
        ):
            return None

        request = CalculateFare.Request()
        request.building_name = building_name
        request.luggage_weight = float(luggage_weight)

        self.get_logger().info('요금 계산 요청 전송')

        future = self.fare_client.call_async(request)

        return self.wait_for_future(
            future,
            '요금 계산'
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            '\n========== 이동 피드백 ==========\n'
            f'상태: {feedback.status}\n'
            f'진행률: {feedback.progress:.1f}%\n'
            f'실제 이동 거리: '
            f'{feedback.traveled_distance:.2f}m\n'
            f'남은 거리: '
            f'{feedback.remaining_distance:.2f}m\n'
            '================================'
        )

    def send_goal(
        self,
        building_name,
        luggage_weight,
        pickup_pin
    ):
        self.get_logger().info(
            'go_to_building 액션 서버 연결 대기 중...'
        )

        if not self.action_client.wait_for_server(
            timeout_sec=1.0
        ):
            while rclpy.ok():
                self.get_logger().info(
                    '액션 서버 연결을 기다립니다...'
                )

                if self.action_client.wait_for_server(
                    timeout_sec=2.0
                ):
                    break

            if not rclpy.ok():
                return None

        goal_msg = GoToBuilding.Goal()
        goal_msg.building_name = building_name
        goal_msg.luggage_weight = float(luggage_weight)
        goal_msg.pickup_pin = pickup_pin

        self.get_logger().info(
            f'목적지 요청 전송: {building_name}'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        goal_handle = self.wait_for_future(
            send_goal_future,
            '목적지 요청 승인'
        )

        if goal_handle is None:
            return None

        if not goal_handle.accepted:
            self.get_logger().error(
                '서버에서 목적지 요청을 거절했습니다.'
            )
            return None

        self.get_logger().info(
            '목적지 요청 승인! 이동 피드백을 기다립니다.'
        )

        result_future = goal_handle.get_result_async()

        wrapped_result = self.wait_for_future(
            result_future,
            '이동 결과'
        )

        if wrapped_result is None:
            return None

        result = wrapped_result.result

        self.get_logger().info(
            f'액션 상태 코드: {wrapped_result.status}'
        )

        if result.success:
            self.get_logger().info(
                f'이동 성공: {result.message}\n'
                f'최종 이동 거리: '
                f'{result.traveled_distance:.2f}m\n'
                f'최종 요금: {result.final_fare}원'
            )
        else:
            self.get_logger().error(
                f'이동 실패: {result.message}'
            )

        return result

    def request_pickup_verification(self, entered_pin):
        if not self.wait_for_service(
            self.pickup_client,
            'verify_pickup'
        ):
            return None

        request = VerifyPickup.Request()
        request.entered_pin = entered_pin

        self.get_logger().info(
            '짐 수령 인증 요청 전송'
        )

        future = self.pickup_client.call_async(request)

        return self.wait_for_future(
            future,
            '짐 수령 인증'
        )


def get_pickup_pin():
    while True:
        pin = input(
            '짐 수령에 사용할 4자리 인증번호를 설정하세요: '
        ).strip()

        if len(pin) == 4 and pin.isascii() and pin.isdigit():
            return pin

        print('인증번호는 숫자 4자리로 입력해주세요.')


def main(args=None):
    rclpy.init(args=args)

    node = GoalsettingClient()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # ROS 콜백은 별도 스레드에서 처리
    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )
    executor_thread.start()

    buildings = {
        '1': '테크노 빌딩',
        '2': '종합 정보 센터',
        '3': '도서관'
    }

    try:
        print('\n===== 목적지 선택 =====')
        for number, name in buildings.items():
            print(f'{number}. {name}')

        choice = input(
            '이동할 목적지 번호를 선택하세요: '
        ).strip()

        if choice not in buildings:
            print('올바른 목적지 번호가 아닙니다.')
            return

        building_name = buildings[choice]

        print('\n===== 짐 무게별 기본 요금 =====')
        print('0~3kg: 0원')
        print('3kg 초과~6kg: 300원')
        print('6kg 초과~10kg: 500원')
        print('거리 요금: 50m당 100원')
        print('================================')

        try:
            luggage_weight = float(
                input('짐 무게를 입력하세요(kg): ').strip()
            )
        except ValueError:
            print('올바른 숫자를 입력해주세요.')
            return

        if not 0.0 <= luggage_weight <= 10.0:
            print('짐 무게는 0kg 이상 10kg 이하여야 합니다.')
            return

        # 1. 요금 계산
        fare_response = node.request_fare(
            building_name,
            luggage_weight
        )

        if fare_response is None:
            print('요금 계산 응답을 받지 못했습니다.')
            return

        if not fare_response.accepted:
            print(f'요금 계산 거절: {fare_response.message}')
            return

        print('\n========== 운반 요금 안내 ==========')
        print(f'목적지: {building_name}')
        print(f'짐 무게: {luggage_weight}kg')
        print(f'예상 거리: {fare_response.estimated_distance:.2f}m')
        print(f'기본 요금: {fare_response.base_fee}원')
        print(f'거리 요금: {fare_response.distance_fee}원')
        print(f'총 예상 요금: {fare_response.total_fee}원')
        print('====================================')

        approval = input(
            '이 요금으로 이동하시겠습니까? (y/n): '
        ).strip().lower()

        if approval != 'y':
            print('이동 요청을 취소했습니다.')
            return

        pickup_pin = get_pickup_pin()

        # 2. 액션 요청 및 피드백/결과 대기
        result = node.send_goal(
            building_name,
            luggage_weight,
            pickup_pin
        )

        if result is None or not result.success:
            print('이동이 완료되지 않았습니다.')
            return

        # 3. 목적지 도착 후 인증번호 확인
        while rclpy.ok():
            entered_pin = input(
                '짐 수령 인증번호 4자리를 입력하세요: '
            ).strip()

            if len(entered_pin) != 4 or not (
                entered_pin.isascii() and entered_pin.isdigit()
            ):
                print('인증번호는 숫자 4자리로 입력해주세요.')
                continue

            response = node.request_pickup_verification(
                entered_pin
            )

            if response is None:
                print('인증 응답을 받지 못했습니다.')
                continue

            print(response.message)

            if response.verified:
                break

    except KeyboardInterrupt:
        print('\n클라이언트를 종료합니다.')

    finally:
        executor.shutdown()
        executor_thread.join(timeout=2.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
