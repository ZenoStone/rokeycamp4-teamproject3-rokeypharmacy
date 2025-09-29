import os
import time
import warnings
import json
import sys
import cv2
import numpy as np
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
from gtts import gTTS
from playsound import playsound
from scipy.signal import resample
from ament_index_python.packages import get_package_share_directory

import STT
import MicController

# --- [수정] 설정 파일 로딩 함수 통합 ---
def load_json_from_share(package_name, filename):
    """ROS2 패키지의 share 디렉토리에서 JSON 파일을 찾아 로드하는 범용 함수."""
    try:
        share_dir = get_package_share_directory(package_name)
        json_path = os.path.join(share_dir, filename)
        
        print(f"'{json_path}' 에서 설정 정보를 로드합니다.")
        with open(json_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        print(f"오류: '{filename}' 파일 로딩 실패. ({e})")
        sys.exit(1)

# --- 프로그램 시작 시 모든 설정 정보 로드 ---
ALL_MEDICINES = load_json_from_share('meditts', 'medicines.json')
TTS_MESSAGES = load_json_from_share('meditts', 'tts_messages.json')

# 로드된 정보를 기반으로 딕셔너리 동적 생성
MEDICINE_EXPLANATIONS = {med['id']: med['explanation'] for med in ALL_MEDICINES}
MEDICINE_KOREAN_NAMES = {med['id']: med['name_kr'] for med in ALL_MEDICINES}

# [수정] 생성된 음성 파일 추적을 위한 Set
GENERATED_SOUND_FILES = set()

# 사용자 종료 예외 정의
class UserExitException(Exception):
    """사용자가 '취소'/'종료'를 말할 때 사용하는 예외"""
    pass

# --- 환경 설정 ---
load_dotenv(dotenv_path=".env")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

MODEL_PATH = "/home/rokey/hubtest_ws/src/meditts/meditts/hello_rokey_8332_32.tflite"
MODEL_NAME = os.path.splitext(os.path.basename(MODEL_PATH))[0]


# --- [수정] text_to_speech 함수 개선 ---
def text_to_speech(key, *args):
    """JSON에서 메시지 키를 찾아 음성으로 변환하고 재생합니다."""
    # JSON에서 키에 해당하는 문장을 가져옵니다.
    text_template = TTS_MESSAGES.get(key)
    if not text_template:
        print(f"경고: TTS 메시지 키 '{key}'를 찾을 수 없습니다.")
        return

    # 추가 인자(args)가 있으면 문장의 {} 부분을 채웁니다.
    text_to_say = text_template.format(*args)
    
    # 파일 이름은 키를 기반으로 생성합니다.
    filename = f"{key}.mp3"
    GENERATED_SOUND_FILES.add(filename) # 삭제 목록에 추가

    if not os.path.exists(filename):
        print(f"'{filename}' 생성 중: \"{text_to_say}\"")
        tts = gTTS(text_to_say, lang='ko')
        tts.save(filename)
    playsound(filename)


# --- Wake Word 클래스 (기존과 동일) ---
class WakeupWord:
    # ... (내용 변경 없음)
    def __init__(self, buffer_size):
        from openwakeword.model import Model
        self.model = Model(wakeword_models=[MODEL_PATH])
        self.model_name = MODEL_NAME
        self.stream = None
        self.buffer_size = buffer_size

    def set_stream(self, stream):
        self.stream = stream

    def is_wakeup(self):
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs.get(self.model_name, 0.0)
        return confidence > 0.15

# --- 의약품 추천 클래스 (기존과 동일) ---
class ExtractMedicine:
    # ... (내용 변경 없음)
    def __init__(self):
        self.llm = ChatOpenAI(
            model="gpt-4", temperature=0.1, openai_api_key=OPENAI_API_KEY
        )
        
        otc_medicines = [m for m in ALL_MEDICINES if m['type'] == 'otc']
        prescription_medicines = [m for m in ALL_MEDICINES if m['type'] == 'prescription']

        otc_list_str = "\n".join([f"- {m['id']}: {m['symptoms']}" for m in otc_medicines])
        prescription_ids_str = ', '.join([f"'{m['id']}'" for m in prescription_medicines])
        output_format_str = ','.join([f"{m['id']}:bool" for m in ALL_MEDICINES])
        
        prompt_content = f"""
        당신은 처방전 없이 구매 가능한 일반 의약품 추천 전문가입니다. 
        사용자가 말하는 증상을 듣고, 추천 가능한 의약품 목록에 해당하는 경우에만 약을 추천합니다.

        <매우 중요한 규칙>
        - 이 시나리오는 사용자가 "처방전이 없는 경우"이므로, 처방전이 필요한 의약품은 절대 추천해서는 안 됩니다.
        - 따라서 {prescription_ids_str}는 사용자의 증상과 관계없이 무조건 False로 설정해야 합니다.

        <추천 가능한 의약품 목록 및 관련 증상>
        {otc_list_str}

        <출력 형식>
        - 반드시 다음 형식에 맞춰 각 의약품의 필요 여부를 True 또는 False로만 답해야 합니다.
        - 다른 설명 없이, 아래 형식만 정확히 지켜 한 줄로 출력해주세요.
        {output_format_str}

        <사용자 증상>
        "{{user_input}}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm

    def extract(self, user_symptom):
        response = self.lang_chain.invoke({"user_input": user_symptom})
        response_text = response.content.strip()
        print(f"LLM 응답: {response_text}")

        medicine_flags = {}
        try:
            parts = response_text.split(',')
            for part in parts:
                key, value = part.split(':')
                medicine_flags[key.strip()] = value.strip().lower() == 'true'
            return medicine_flags
        except Exception as e:
            warnings.warn(f"LLM 응답 파싱 실패: {e}\n응답: {response_text}")
            return {med['id']: False for med in ALL_MEDICINES}

# --- 사용자 응답 처리 (수정: TTS 호출 방식 변경) ---
def get_user_response(stt_instance):
    while True:
        user_answer = stt_instance.speech2text().lower()

        if any(keyword in user_answer for keyword in ["취소", "종료"]):
            print("사용자가 종료를 요청했습니다.")
            text_to_speech("exit_prompt")
            raise UserExitException()

        if any(keyword in user_answer for keyword in ["예", "네", "응", "어", "그래", "줘", "yay", "yeah"]):
            return "yes"
        elif any(keyword in user_answer for keyword in ["아니요", "아니", "싫어", "아뇨"]):
            return "no"
        else:
            text_to_speech("unclear_answer")


# --- 약품 제공 공통 함수 (수정: TTS 호출 방식 변경) ---
def confirm_and_provide_medicines(medicines_to_provide, stt_instance, question_key):
    if not medicines_to_provide:
        text_to_speech("no_recommendation")
        return False 

    korean_names = [MEDICINE_KOREAN_NAMES.get(m, m) for m in medicines_to_provide]
    medicines_text = ", ".join(korean_names)
    
    text_to_speech(question_key, medicines_text) # 예: text_to_speech("ask_if_provide_recommendation", "타이레놀")
    
    user_choice = get_user_response(stt_instance)
    
    if user_choice == "yes":
        text_to_speech("provide_medicine", medicines_text)
        time.sleep(0.5)
        return True
    else: # "no"
        text_to_speech("returning_to_previous")
        return False


# --- 증상 기반 추천 함수 (수정: TTS 호출 방식 변경) ---
def run_drug_store(stt_instance):
    while True:
        user_symptom = ""
        for attempt in range(2):
            key = "symptom_request" if attempt == 0 else "retry_symptom_request"
            text_to_speech(key)
            
            symptom_text = stt_instance.speech2text()

            if any(keyword in symptom_text.lower() for keyword in ["취소", "종료"]):
                 print("사용자가 종료를 요청했습니다.")
                 text_to_speech("exit_prompt")
                 raise UserExitException()

            if symptom_text and len(symptom_text.strip()) > 1:
                user_symptom = symptom_text.strip()
                print(f"인식된 증상: {user_symptom}")
                break
            else:
                print(f"시도 {attempt + 1}: 증상 음성 인식에 실패했거나 내용이 너무 짧습니다.")
        
        if not user_symptom:
            print("최종적으로 증상 인식에 실패했습니다.")
            text_to_speech("recognition_failed")
            return None

        extractor = ExtractMedicine()
        medicine_flags = extractor.extract(user_symptom)
        
        recommended_medicines = [m for m, needed in medicine_flags.items() if needed]
        
        if confirm_and_provide_medicines(recommended_medicines, stt_instance, "ask_if_provide_recommendation"):
            return medicine_flags

# --- QR 코드 처리 함수들 (수정: TTS 호출 방식 변경) ---
ROI_SIZE = 320
FIXED_DEVICE = 8
STATUS_COLOR = (80, 255, 80)

def open_camera():
    # ... (내용 변경 없음)
    cap = cv2.VideoCapture(FIXED_DEVICE)
    if not cap.isOpened():
        print(f"웹캠을 열 수 없습니다. /dev/video{FIXED_DEVICE} 장치 없음 또는 초기화 실패")
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, frame = cap.read()
    if ok and frame is not None:
        print(f"사용 카메라: /dev/video{FIXED_DEVICE} ({frame.shape[1]}x{frame.shape[0]})")
        return cap
    cap.release()
    print("카메라에서 프레임을 읽어올 수 없습니다.")
    return None

def clip_roi(frame, x1, y1, x2, y2):
    # ... (내용 변경 없음)
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return None, (x1, y1, x2, y2)
    return frame[y1:y2, x1:x2], (x1, y1, x2, y2)

def scan_qr_with_roi():
    cap = open_camera()
    if cap is None: return None
    qr = cv2.QRCodeDetector()
    decoded_data = None
    print("QR 스캐너 실행. ESC 키로 종료.")
    
    timeout = 30 
    start_time = time.time()

    while True:
        if time.time() - start_time > timeout:
            print(f"{timeout}초 동안 QR코드가 인식되지 않아 종료합니다.")
            text_to_speech("qr_timeout")
            break
        # ... (이하 카메라 처리 로직은 기존과 동일)
        ret, frame = cap.read()
        if not ret or frame is None:
            status = "프레임 읽기 실패"
            time.sleep(0.1)
            continue

        h, w, _ = frame.shape
        cx, cy = w // 2, h // 2
        x1, y1 = cx - ROI_SIZE // 2, cy - ROI_SIZE // 2
        x2, y2 = cx + ROI_SIZE // 2, cy + ROI_SIZE // 2
        roi, (rx1, ry1, rx2, ry2) = clip_roi(frame, x1, y1, x2, y2)
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), STATUS_COLOR, 2)
        
        remaining_time = timeout - int(time.time() - start_time)
        cv2.putText(frame, f"Time left: {remaining_time}s", (rx1, ry1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        if roi is not None and roi.size > 0:
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            try:
                data_str, points, _ = qr.detectAndDecode(gray_roi)
                if data_str:
                    status = f"QR 코드 인식 성공!"
                    decoded_data = data_str
                    if points is not None:
                        p = points.reshape(-1, 2).astype(int)
                        p[:, 0] += rx1
                        p[:, 1] += ry1
                        cv2.polylines(frame, [p], True, (0, 200, 255), 2)
                    cv2.imshow('QR Code Scanner', frame)
                    cv2.waitKey(1000)
                    break
            except Exception as e:
                status = f"QR 해석 오류: {e}"
        
        cv2.putText(frame, "QR코드를 사각형 안에 위치시켜 주세요 (ESC: 종료)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 220, 255), 2)
        cv2.imshow('QR Code Scanner', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("사용자가 스캔을 취소했습니다.")
            decoded_data = None
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return decoded_data

def handle_prescription_qr(stt_instance):
    while True:
        text_to_speech("prescription_instructions")
        qr_data_string = scan_qr_with_roi()

        final_flags = {med['id']: False for med in ALL_MEDICINES}

        if qr_data_string:
            print(f"인식된 QR코드 데이터: {qr_data_string}")
            try:
                parsed_data = json.loads(qr_data_string)
                for key in final_flags.keys():
                    if key in parsed_data and isinstance(parsed_data.get(key), bool):
                        final_flags[key] = parsed_data[key]
                
                text_to_speech("valid_prescription")
                
                prescribed_medicines = [m for m, needed in final_flags.items() if needed]
                
                if confirm_and_provide_medicines(prescribed_medicines, stt_instance, "ask_if_provide_prescription"):
                    return final_flags
            
            except json.JSONDecodeError:
                text_to_speech("invalid_prescription")
        
        else:
            print("사용자가 스캔을 취소했거나 QR코드를 찾지 못했습니다.")
            return None


# --- 메인 실행 블록 (수정: TTS 호출 방식 및 파일 정리 로직 변경) ---
if __name__ == "__main__":
    final_decision_data = None
    Mic = None
    try:
        Mic = MicController.MicController()
        Mic.open_stream()

        wakeup = WakeupWord(Mic.config.buffer_size)
        wakeup.set_stream(Mic.stream)
        print("대기 중... 'hello rokey'를 말씀해주세요.")
        
        while not wakeup.is_wakeup():
            pass
        
        print("호출어가 감지되었습니다!")

        stt = STT.STT(OPENAI_API_KEY)
        
        try:
            text_to_speech("prescription_query")
            user_response = get_user_response(stt)

            if user_response == "yes":
                final_decision_data = handle_prescription_qr(stt)
            else: # "no"
                final_decision_data = run_drug_store(stt)
            
            if final_decision_data:
                print("프로세스가 정상적으로 완료되었습니다.")

        except UserExitException:
            print("사용자 요청으로 프로세스를 중단했습니다.")
            final_decision_data = None
    
    except Exception as e:
        print(f"오류가 발생하여 프로그램을 종료합니다: {e}")
        
    finally:
        if Mic and Mic.stream and Mic.stream.is_active():
             Mic.close_stream()

        if final_decision_data:
            try:
                file_path = os.path.join(os.path.expanduser("~"), "decision.json")
                
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(final_decision_data, f, ensure_ascii=False, indent=4)
                print(f"✅ 최종 결과가 '{file_path}' 경로에 저장되었습니다.")

            except Exception as e:
                print(f"JSON 파일 저장 중 오류 발생: {e}")
        else:
            print("⚠️ 'decision.json'을 생성할 데이터가 없거나 프로세스가 중단되었습니다.")

        # [수정] 생성된 음성 파일 목록을 기반으로 파일 삭제
        for sound_file in GENERATED_SOUND_FILES:
            if os.path.exists(sound_file):
                try:
                    os.remove(sound_file)
                except Exception as e:
                    print(f"파일 삭제 오류 '{sound_file}': {e}")
        print("임시 음성 파일들을 삭제했습니다.")
