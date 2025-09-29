import os
import sys
import json
import subprocess
import rclpy
from rclpy.node import Node
from pharmacy_interface.srv import MediCine
from gtts import gTTS
from playsound import playsound
# [수정] ROS2 패키지 경로를 찾기 위한 라이브러리 임포트
from ament_index_python.packages import get_package_share_directory

def text_to_speech(text, filename="temp_tts.mp3"):
    """텍스트를 음성으로 변환하고 재생한 뒤, 파일을 삭제합니다."""
    try:
        print(f"TTS 생성 중: \"{text}\"")
        tts = gTTS(text=text, lang='ko')
        tts.save(filename)
        playsound(filename)
    except Exception as e:
        print(f"TTS 실행 중 오류 발생: {e}")
    finally:
        if os.path.exists(filename):
            os.remove(filename)

class MedicineSenderNode(Node):
    def __init__(self):
        super().__init__('medicine_sender_node')
        self.client = self.create_client(MediCine, '/dsr01/order_medicines')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버를 기다리는 중...')
        self.get_logger().info('서비스 서버에 연결되었습니다.')
        

    def run_main_script_and_send_order(self):
        target_dir = os.path.expanduser("~") 
        decision_json_path = os.path.join(target_dir, "decision.json")

        # [수정] medicines.json 파일 경로를 ROS2 표준 방식으로 찾습니다.
        try:
            share_dir = get_package_share_directory('meditts')
            medicines_json_path = os.path.join(share_dir, 'meditts', 'medicines.json')
            # STT_QR_connect.py 스크립트의 경로도 share 디렉토리를 기준으로 찾도록 변경할 수 있으나,
            # 여기서는 기존 방식을 유지하되 medicines.json 경로만 수정합니다.
            base_dir = os.path.dirname(__file__)
            script_path = os.path.join(base_dir, 'STT_QR_connect.py')
        except Exception as e:
            self.get_logger().error(f"패키지 경로 설정 실패: {e}")
            return

        if os.path.exists(decision_json_path):
            try:
                os.remove(decision_json_path)
                self.get_logger().info(f"이전 '{decision_json_path}' 파일을 삭제했습니다.")
            except OSError as e:
                self.get_logger().error(f"파일 삭제에 실패했습니다: {e}")
                return

        self.get_logger().info(f"'{script_path}' 스크립트 실행 중...")
        try:
            subprocess.run([sys.executable, script_path], check=True, text=True)
            self.get_logger().info("스크립트 실행 완료.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"스크립트가 오류와 함께 종료되었습니다: {e}")
            return

        if not os.path.exists(decision_json_path):
            self.get_logger().info("사용자 상호작용이 완료되지 않아 서비스 요청을 보내지 않습니다.")
            return

        self.get_logger().info(f"'{decision_json_path}' 파일 읽는 중...")
        try:
            with open(decision_json_path, 'r', encoding='utf-8') as f:
                decision_data = json.load(f)
            
            with open(medicines_json_path, 'r', encoding='utf-8') as f:
                all_medicines_info = json.load(f)
                medicines_dict = {med['id']: med for med in all_medicines_info}

        except Exception as e:
            self.get_logger().error(f"JSON 파일 읽기 실패: {e}")
            return

        # --- (이하 서비스 요청 및 TTS 안내 로직은 동일) ---
        request = MediCine.Request()
        request.penzal = decision_data.get('penzal', False)
        request.sky = decision_data.get('sky', False)
        request.tg = decision_data.get('tg', False)
        request.zaide = decision_data.get('zaide', False)
        request.famotidine = decision_data.get('famotidine', False)
        request.somnifacient = decision_data.get('somnifacient', False)
        request.allergy = decision_data.get('allergy', False)
        
        self.get_logger().info("서비스 요청 전송 중...")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            self.get_logger().info(f"서버 응답: {response}")

            if response:
                precautions_list = []
                for med_id, is_selected in decision_data.items():
                    if is_selected:
                        med_info = medicines_dict.get(med_id)
                        if med_info:
                            med_name = med_info.get('name_kr', med_id)
                            explanation = med_info.get('explanation', '별도 주의사항이 없습니다.')
                            precautions_list.append(f"{med_name}의 복용 시 주의사항입니다. {explanation}")
                
                if precautions_list:
                    full_precautions_text = " ".join(precautions_list)
                    text_to_speech(full_precautions_text, "precautions.mp3")

        except Exception as e:
            self.get_logger().error(f'서비스 요청 중 예외 발생: {e}')
        
        self.get_logger().info(f"'{decision_json_path}' 파일을 삭제하지 않고 유지합니다.")

def main(args=None):
    rclpy.init(args=args)
    node = MedicineSenderNode()
    try:
        # 노드가 종료되지 않고 계속 다음 작업을 기다리도록 무한 루프를 추가합니다.
        while rclpy.ok():
            node.run_main_script_and_send_order()
            
    except KeyboardInterrupt:
        # Ctrl+C를 누르면 루프를 빠져나와 정상적으로 종료됩니다.
        node.get_logger().info("사용자 인터럽트로 노드를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()