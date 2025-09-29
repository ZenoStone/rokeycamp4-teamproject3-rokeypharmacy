import json
from typing import List, Dict, Union

class WaypointManager:
    def __init__(self, json_path: str):
        self.json_path = json_path
        self.waypoints: Dict[str, Dict] = {}
        self.groups: Dict[str, List[str]] = {}
        self.load()

    def load(self):
        try:
            with open(self.json_path, 'r') as f:
                data = json.load(f)
            for wp in data.get("waypoints", []):
                self.waypoints[wp["name"]] = wp
            self.groups = data.get("groups", {})
        except FileNotFoundError:
            print(f"[오류] 파일 '{self.json_path}'을 찾을 수 없습니다.")
        except json.JSONDecodeError:
            print(f"[오류] 파일 '{self.json_path}'의 형식이 잘못되었습니다.")
        except Exception as e:
            print(f"[오류] 파일을 읽는 중에 예기치 않은 오류가 발생했습니다: {e}")

    def get(self, name: str) -> Union[Dict, None]:
        return self.waypoints.get(name)

    def get_pos(self, name: str):
        from DR_common2 import posx, posj
        wp = self.get(name)
        if wp is None:
            print(f"[경고] '{name}' 좌표 없음")
            return None
        return posx(wp["value"]) if wp["type"] == "task" else posj(wp["value"])

    def get_group(self, group_name: str):
        return [self.get_pos(name) for name in self.groups.get(group_name, [])]
