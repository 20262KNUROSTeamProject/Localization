import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

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

    def request_fare(self, building_name, luggage_weight):
        self.get_logger().info('요금 계산 서버 연결 대기 중...')

        self.fare_client.wait_for_service()

        request = CalculateFare.Request()
        request.building_name = building_name
        request.luggage_weight = float(luggage_weight)

        self.get_logger().info('요금 계산 요청 전송')

        self.fare_future = self.fare_client.call_async(request)
        self.fare_future.add_done_callback(
            lambda future: self.fare_response_callback(
                future, building_name, luggage_weight
            )
        )

    def fare_response_callback(self, future, building_name, luggage_weight):
        response = future.result()

        if not response.accepted:
            self.get_logger().warn(response.message)
            rclpy.shutdown()
            return

        print('\n========== 운반 요금 안내 ==========')
        print(f'목적지: {building_name}')
        print(f'짐 무게: {luggage_weight}kg')
        print(f'예상 거리: {response.estimated_distance:.2f}m')
        print(f'기본 요금: {response.base_fee}원')
        print(f'거리 요금: {response.distance_fee}원')
        print(f'총 예상 요금: {response.total_fee}원')
        print('====================================')

        approval = input('이 요금으로 이동하시겠습니까? (y/n): ').strip().lower()

        if approval == 'y':
            self.get_logger().info('이동을 승인했습니다.')

            while True:
                pickup_pin = input(
                    '짐 수령에 사용할 4자리 인증번호를 설정하세요: '
                ).strip()

                if (
                    len(pickup_pin) == 4
                    and pickup_pin.isascii()
                    and pickup_pin.isdigit()
                ):
                    break

                print('인증번호는 숫자 4자리로 입력해주세요.')

            self.send_goal(
                building_name,
                luggage_weight,
                pickup_pin
            )
        else:
            self.get_logger().info('이동 요청을 취소했습니다.')
            rclpy.shutdown()

    def send_goal(self, building_name, luggage_weight, pickup_pin):
        self.get_logger().info(f'서버 연결 대기 중...')

        self.action_client.wait_for_server()

        goal_msg = GoToBuilding.Goal()
        goal_msg.building_name = building_name
        goal_msg.luggage_weight = float(luggage_weight)
        goal_msg.pickup_pin = pickup_pin

        self.get_logger().info(f'목적지 요청 전송: {building_name}')

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(
                future, luggage_weight
            )
        )

    def goal_response_callback(self, future, luggage_weight ):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn(
                '목적지 요청이 거절되었습니다. '
                '건물 이름과 짐 무게를 확인해주세요.'
            )
            building_name = input(
                '이동할 건물 이름을 다시 입력하세요: '
            ).strip()
        
            try:
                luggage_weight = float(
                    input('짐 무게를 다시 입력하세요(kg): ').strip()
                )

            except ValueError:
                self.get_logger().error('올바른 숫자를 입력해주세요.')
                rclpy.shutdown()
                return

            if luggage_weight < 0 or luggage_weight > 10:
                self.get_logger().error(
                    '짐 무게는 0kg 이상 10kg 이하여야 합니다.'
                )
                rclpy.shutdown()
                return

            self.request_fare(building_name, luggage_weight)
            return
        
        self.get_logger().info('목적지 요청이 승인되었습니다.')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
    
        self.get_logger().info(
            f'이동 상태: 이동 중\n'
            f'진행률: {feedback.progress:.1f}%\n'
            f'실제 이동 거리: {feedback.traveled_distance:.2f}m\n'
            f'남은 거리: {feedback.remaining_distance:.2f}m'
        )

    def result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info(result.message)
            self.request_pickup_verification()
        else:
            self.get_logger().warn(result.message)
            rclpy.shutdown()

    def request_pickup_verification(self):
        while True:
            entered_pin = input(
                '짐 수령 인증번호 4자리를 입력하세요: '
            ).strip()

            if (
                len(entered_pin) == 4
                and entered_pin.isascii()
                and entered_pin.isdigit()
            ):
                break

            print('인증번호는 숫자 4자리로 입력해주세요.')

        self.get_logger().info('짐 수령 인증 요청 중...')
        self.pickup_client.wait_for_service()

        request = VerifyPickup.Request()
        request.entered_pin = entered_pin

        self.pickup_future = self.pickup_client.call_async(request)
        self.pickup_future.add_done_callback(
            self.pickup_response_callback
        )

    def pickup_response_callback(self, future):
        response = future.result()

        if response.verified:
            self.get_logger().info(response.message)
            rclpy.shutdown()
        else:
            self.get_logger().warn(response.message)
            self.request_pickup_verification()

def main(args=None):
    rclpy.init(args=args)

    buildings = {
        '1': '테크노 빌딩',
        '2': '종합 정보 센터',
        '3': '도서관'
    }

    while True:
        print('\n===== 목적지 선택 =====')
        for number, name in buildings.items():
            print(f'{number}. {name}')
        print('4. 기타 (건물명 직접 입력)')

        choice = input('이동할 목적지 번호를 선택하세요: ').strip()

        if choice in buildings:
            building_name = buildings[choice]
            break

        elif choice == '4':
            building_name = input(
                '이동할 건물 이름을 입력하세요: '
            ).strip()

            if building_name not in buildings.values():
                print(
                    f'등록되지 않은 건물입니다: {building_name}'
                )
                print('다시 목적지를 선택해주세요.')
                continue

            break

        else:
            print('올바른 번호를 선택해주세요.')

    print(f'\n선택한 목적지: {building_name}')

    print('\n===== 짐 무게별 기본 요금 안내 =====')
    print('0~3kg: 0원')
    print('3kg 초과~6kg: 300원')
    print('6kg 초과~10kg: 500원')
    print('10kg 초과: 운반 불가')
    print('거리 요금: 50m당 100원 (올림 계산)')
    print('최대 적재 중량: 10kg')
    print('====================================\n')


    try:
        luggage_weight = float(
            input('짐 무게를 입력하세요(kg): ').strip()
        )
    except ValueError:
        print('올바른 숫자를 입력해주세요.')
        rclpy.shutdown()
        return

    if luggage_weight < 0 or luggage_weight > 10:
        print('짐 무게는 0kg 이상 10kg 이하여야 합니다.')
        rclpy.shutdown()
        return

    node = GoalsettingClient()

    # 요금 계산 요청
    node.request_fare(building_name, luggage_weight)

    rclpy.spin(node)

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
