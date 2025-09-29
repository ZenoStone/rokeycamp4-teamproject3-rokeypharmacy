import rclpy
from rclpy.node import Node
# 클라이언트와 동일한 서비스 인터페이스를 임포트합니다.
from pharmacy_interface.srv import MediCine
from std_msgs.msg import String


import DR_init

ROBOT_ID = "dsr01"
DR_init.__dsr__id = ROBOT_ID


class MedicineServerNode(Node):
    """
    클라이언트로부터 약품 정보를 받아 상태를 출력하고 응답하는 서비스 서버.
    """
    def __init__(self):
        super().__init__('medicine_server_node',  namespace=ROBOT_ID)


        # 이동 명령 발행 토픽
        self.go_pub = self.create_publisher(String, '/go_main', 10)

        
        # 'order_medicines' 라는 이름으로 MediCine 타입의 서비스를 생성합니다.
        # 요청이 오면 handle_request 콜백 함수가 실행됩니다.
        self.srv = self.create_service(
            MediCine, 
            'order_medicines', 
            self.handle_request)
        self.get_logger().info('약품 주문 서비스 서버가 준비되었습니다.')

    def handle_request(self, request, response):
        """서비스 요청을 처리하는 콜백 함수."""
        self.get_logger().info('새로운 약품 주문 요청을 받았습니다:')
        
        # 요청받은 각 약품의 상태(bool)를 터미널에 출력합니다.
        self.get_logger().info('--- 요청된 약품 상태 ---')
        self.get_logger().info(f"  - Penzal: {request.penzal}")
        self.get_logger().info(f"  - Sky: {request.sky}")
        self.get_logger().info(f"  - Tg: {request.tg}")
        self.get_logger().info(f"  - Zaide: {request.zaide}")
        self.get_logger().info(f"  - Famotidine: {request.famotidine}")
        self.get_logger().info(f"  - Somnifacient: {request.somnifacient}")
        self.get_logger().info(f"  - Allergy: {request.allergy}")
            
        # 응답 메시지를 설정합니다. (요청받은 값을 그대로 되돌려줌)
        response.penzal = request.penzal
        response.sky = request.sky
        response.tg = request.tg
        response.zaide = request.zaide
        response.famotidine = request.famotidine
        response.somnifacient = request.somnifacient
        response.allergy = request.allergy
        

        # 서비스 요청 싹다 읽어본 다음 True인 것만 결과로 토픽 메세지로 쏘기
     
        md_list_bool = [response.penzal, response.sky, response.tg, response.zaide, response.famotidine, response.somnifacient, response.allergy]
        md_list_name = ['penzal', 'sky', 'tg', 'zaide', 'famotidine', 'somnifacient', 'allergy']
        msg_result = ''

        for i in range(7):
            if md_list_bool[i] == 1:
                msg_result = md_list_name[i]


        msg = String()
        msg.data = msg_result
        self.go_pub.publish(msg)


        # medicine_send4로 서비스 응답 보내기
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MedicineServerNode()
    try:
        rclpy.spin(node) # 노드가 종료될 때까지 계속 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
