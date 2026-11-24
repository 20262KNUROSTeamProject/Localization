
import asyncio
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from goalsetting_interfaces.action import GoToBuilding
from goalsetting_interfaces.srv import CalculateFare
from goalsetting_interfaces.srv import VerifyPickup


class GoalsettingServer(Node):

    def __init__(self):
        super().__init__('goalsetting_server')

        self.building_positions = {
            '테크노 빌딩': (1.9018, 4.1184, 0.0),
            '종합 정보 센터': (6.0569, 2.5835, 3.09),
            '도서관': (5.1261, -1.7043, 1.590),
        }

        self.action_group = ReentrantCallbackGroup()
        self.nav_group = ReentrantCallbackGroup()

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.nav_group
        )

        # 무게별 Nav2 속도 제한 Publisher 
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            '/speed_limit',
            10
        )

        self.action_server = ActionServer (
            self,
            GoToBuilding,
            'go_to_building',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group
        )

        # 요금
        self.max_weight = 10.0
        self.distance_unit = 50.0
        self.distance_fee = 100

        # 현 로봇 위치
        self.current_x = None
        self.current_y = None
        
        # 현재 운반 중인 짐 수령 인증번호
        self.current_pickup_pin = None

        # 목적지 도착 및 짐 수령 대기 상태
        self.pickup_ready = False

        # 실제 이동 거리 측정
        self.is_tracking_movement = False
        self.last_x = None
        self.last_y = None
        self.traveled_distance = 0.0

        # 현 위치 구독
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # 요금 계산 Service
        self.fare_service = self.create_service(
            CalculateFare,
            'calculate_fare',
            self.calculate_fare_callback
        )

        # 짐 수령 인증번호 확인 Service
        self.pickup_service = self.create_service(
            VerifyPickup,
            'verify_pickup',
            self.verify_pickup_callback
        )
        self.get_logger().info('건물 안내 및 운반 서버가 실행되었습니다.')

    def pose_callback(self, msg):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # 거리 누적
        if self.is_tracking_movement:
            if self.last_x is not None and self.last_y is not None:
                distance = math.sqrt(
                    (new_x - self.last_x) ** 2
                    + (new_y - self.last_y) ** 2
                )
                
                self.traveled_distance += distance
            self.last_x = new_x
            self.last_y = new_y
        
        self.current_x = new_x
        self.current_y = new_y

    def get_base_fee(self, weight):
        if weight <= 3.0:
            return 0
        elif weight <= 6.0:
            return 300
        else:
            return 500

    def get_speed_limit_percentage(self, weight):
        if weight <= 3.0:
            return 100.0
        elif weight <= 6.0:
            return 70.0
        else:
            return 40.0
    
    def publish_speed_limit(self, weight):
        msg = SpeedLimit()
        
        msg.speed_limit = self.get_speed_limit_percentage(weight)
        msg.percentage = True
        
        self.speed_limit_pub.publish(msg)
        
        self.get_logger().info(
            f'짐 무게 {weight}kg → Nav2 속도 제한 '
            f'{msg.speed_limit:.0f}% 적용 요청'
        )

    def reset_speed_limit(self):
        msg = SpeedLimit()
        msg.speed_limit = 0.0
        msg.percentage = False
        self.speed_limit_pub.publish(msg)

    def calculate_fare_callback(self, request, response):
        building_name = self.normalize_building_name(
            request.building_name
        )
        weight = request.luggage_weight

        # 건물 이름 확인
        normalized_positions = {
            self.normalize_building_name(name): (name, pose)
            for name, pose in self.building_positions.items()
        }

        if building_name not in normalized_positions:
            response.accepted = False
            response.message = '등록되지 않은 건물입니다.'
            return response

        # 짐 무게
        if weight < 0 or weight > self.max_weight:
            response.accepted = False
            response.message = '짐 무게는 0kg 이상 10kg 이하여야 합니다.'
            return response

        # 현 로봇 위치 확인
        if self.current_x is None or self.current_y is None:
            response.accepted = False
            response.message = '로봇의 현재 위치를 확인할 수 없습니다.'
            return response

        # 목적지 직선거리 계산
        building, (target_x, target_y, target_yaw) = (
            normalized_positions[building_name]
        )

        distance = math.sqrt(
            (target_x - self.current_x) ** 2
            + (target_y - self.current_y) ** 2
        )

        # 요금 계산
        base_fee = self.get_base_fee(weight)
        distance_fee = math.ceil(
            distance / self.distance_unit
        ) * self.distance_fee

        response.accepted = True
        response.message = f'{building} 목적지 요금 계산 완료'
        response.estimated_distance = float(distance)
        response.base_fee = base_fee
        response.distance_fee = distance_fee
        response.total_fee = base_fee + distance_fee

        self.get_logger().info(
            f'요금 계산 완료: 무게={weight}kg, '
            f'거리={distance:.2f}m, '
            f'총 요금={response.total_fee}원'
        )

        return response

    def verify_pickup_callback(self, request, response):
        # 목적지 도착 여부 확인
        if not self.pickup_ready:
            response.verified = False
            response.message = '아직 목적지에 도착하지 않았습니다.'
            return response

        # 인증번호 일치 확인
        if request.entered_pin == self.current_pickup_pin:
            response.verified = True
            response.message = '인증 성공! 짐 수령이 완료되었습니다.'

            self.get_logger().info('짐 수령 인증 성공')

            # 인증 후 초기화
            self.current_pickup_pin = None
            self.pickup_ready = False

        else:
            response.verified = False
            response.message = '인증번호가 일치하지 않습니다. 다시 입력해주세요.'

        return response

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

        # 짐 무게 검증
        weight = goal_request.luggage_weight

        if weight < 0 or weight > self.max_weight:
            self.get_logger().warn(
                f'잘못된 짐 무게: {weight}kg'
            )
            return GoalResponse.REJECT

        # 짐 수령 인증번호 검증
        pickup_pin = goal_request.pickup_pin

        if (
            len(pickup_pin) != 4
            or not pickup_pin.isascii()
            or not pickup_pin.isdigit()
        ):
            self.get_logger().warn(
                '인증번호는 숫자 4자리여야 합니다.'
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
        feedback.traveled_distance = float(self.traveled_distance)
        feedback.remaining_distance = float(distance)

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

        # 이번 운반의 짐 수령 인증번호 저장
        self.current_pickup_pin = goal_handle.request.pickup_pin
        self.pickup_ready = False

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.traveled_distance = 0.0
                result.final_fare = 0
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

        self.publish_speed_limit(goal_handle.request.luggage_weight)

        initial_distance = [None]

        nav_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda msg: self.nav_feedback_callback(
                goal_handle, initial_distance, msg
            )
        )

        nav_goal_handle = await nav_goal_future

        if not nav_goal_handle.accepted:
            self.reset_speed_limit()
            goal_handle.abort()
            result.success = False
            result.message = 'Nav2 목적지 요청을 거부했습니다.'
            return result

        self.get_logger().info('Nav2 목적지 요청 승인')
        # 실제 이동 거리 측정 시작
        self.traveled_distance = 0.0
        self.last_x = self.current_x
        self.last_y = self.current_y
        self.is_tracking_movement = True

        result_future = nav_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await nav_goal_handle.cancel_goal_async()
                self.is_tracking_movement = False
                self.reset_speed_limit()
                result.success = False
                result.traveled_distance = float(self.traveled_distance)
                result.final_fare = 0
                result.message = '이동이 취소되었습니다.'
                goal_handle.canceled()
                return result

            await asyncio.sleep(0.1)

        nav_result = await result_future

        # 이동 종료 및 실제 이동 거리 확정
        self.is_tracking_movement = False
        self.reset_speed_limit()
        actual_distance = self.traveled_distance

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:

            self.pickup_ready = True
            self.get_logger().info(
                '목적지 도착! 짐 수령 인증을 기다립니다.'
            )
            # 실제 이동 거리 기준 최종 요금 계산
            weight = goal_handle.request.luggage_weight
            
            base_fee = self.get_base_fee(weight)
            
            distance_fee = math.ceil(
                actual_distance / self.distance_unit
            ) * self.distance_fee

            result.traveled_distance = float(actual_distance)
            result.final_fare = base_fee + distance_fee

            goal_handle.succeed()
            result.success = True
            result.message = (
                f'{building}에 도착했습니다. '
                f'실제 이동 거리: {actual_distance:.2f}m, '
                f'최종 요금: {result.final_fare}원'
            )
            self.get_logger().info(result.message)

        else:
            self.current_pickup_pin = None
            self.pickup_ready = False

            goal_handle.abort()
            result.success = False
            result.message = f'{building} 이동에 실패했습니다.'
            self.get_logger().warn(result.message)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GoalsettingServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
