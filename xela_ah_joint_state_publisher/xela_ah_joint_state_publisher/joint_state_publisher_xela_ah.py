#!/usr/bin/env python3
import json
import threading
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# YAML (sudo apt-get install python3-yaml)
import yaml

# URDF parser (sudo apt-get install ros-humble-urdf ros-humble-urdfdom-py)
try:
    from urdf_parser_py.urdf import URDF
except Exception:
    URDF = None

# XACRO (sudo apt-get install ros-humble-xacro)
try:
    import xacro
except Exception:
    xacro = None


NON_FIXED_TYPES = {"revolute", "continuous", "prismatic", "planar"}  # exclude fixed joints


class XelaAhJointStatePublisher(Node):
    def __init__(self):
        super().__init__("xela_ah_joint_state_publisher")

        # -------- Parameters --------
        self.declare_parameter("robot_description_from", "topic")  # "topic" | "param" | "file"
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("robot_description_param", "robot_description")
        self.declare_parameter("urdf_file", "")  # path to .urdf or .xacro

        # YAML config (keep_joints, ordered_keep_joints, initial_positions, etc.)
        self.declare_parameter("config_yaml", "")

        # Publishing & filtering
        self.declare_parameter(
            "keep_joints", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            "ordered_keep_joints", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            "source_list", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            "initial_positions",
            "",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )        
        # self.declare_parameter("keep_joints", [])
        # self.declare_parameter("ordered_keep_joints", [])
        self.declare_parameter("preserve_input_order", True)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("output_topic", "/joint_states")
        # self.declare_parameter("source_list", [])
        # self.declare_parameter("initial_positions", {})
        # self.declare_parameter("use_sim_time", False)

        # Read params
        self.robot_description_from = self._get_str("robot_description_from")
        self.robot_description_topic = self._get_str("robot_description_topic")
        self.robot_description_param = self._get_str("robot_description_param")
        self.urdf_file = self._get_str("urdf_file")
        self.config_yaml = self._get_str("config_yaml")

        self.keep_joints: List[str] = list(self.get_parameter("keep_joints").get_parameter_value().string_array_value)
        self.ordered_keep_joints: List[str] = list(self.get_parameter("ordered_keep_joints").get_parameter_value().string_array_value)
        self.preserve_input_order = self.get_parameter("preserve_input_order").get_parameter_value().bool_value
        self.publish_rate = float(self.get_parameter("publish_rate").get_parameter_value().double_value)
        self.output_topic = self._get_str("output_topic")
        self.source_list: List[str] = list(self.get_parameter("source_list").get_parameter_value().string_array_value)
       
        # self.initial_positions = self._parse_initial_positions(self.get_parameter("initial_positions").value)
        # __init__에서 초기값 파싱 (config_yaml보다 "문자열 파라미터"가 있으면 우선 적용)
        ip_raw = self.get_parameter("initial_positions").get_parameter_value().string_value
        self.initial_positions = {}
        if ip_raw:
            try:
                # JSON 먼저 시도
                parsed = json.loads(ip_raw)
            except Exception:
                # JSON 실패 시 YAML 시도(키:문자열, 값:숫자)
                parsed = yaml.safe_load(ip_raw)
            if isinstance(parsed, dict):
                self.initial_positions = {str(k): float(v) for k, v in parsed.items()}
            else:
                self.get_logger().warn("initial_positions must be a dict JSON/YAML string; ignoring.")

        # -------- Internal --------
        self._urdf_lock = threading.Lock()
        self._known_nonfixed_joints: List[str] = []
        self._last_js_map: Dict[str, Dict[str, float]] = {}  # name -> {"pos","vel","eff"}
        self._last_input_order: List[str] = []
        self._has_urdf = False

        # -------- Pub/Sub --------
        self.pub = self.create_publisher(JointState, self.output_topic, 10)

        self._js_subs = []
        for t in self.source_list:
            self._js_subs.append(self.create_subscription(JointState, t, self._on_joint_state, 50))

        # Load YAML (optional)
        self._load_config_yaml(initial=True)

        # Load URDF / XACRO
        if self.robot_description_from == "topic":
            self.create_subscription(StringMsg, self.robot_description_topic, self._on_robot_description, 10)
            self.get_logger().info(f"Waiting for URDF on topic: {self.robot_description_topic}")
        elif self.robot_description_from == "param":
            desc = self._get_str(self.robot_description_param)
            if desc:
                self._load_urdf(desc)
            else:
                self.get_logger().warn(f"Parameter '{self.robot_description_param}' is empty; no URDF loaded yet.")
        elif self.robot_description_from == "file":
            if not self.urdf_file:
                self.get_logger().error("robot_description_from=='file' but 'urdf_file' is empty.")
            else:
                self._load_urdf(self.urdf_file)
        else:
            self.get_logger().warn(f"Unknown robot_description_from='{self.robot_description_from}', falling back to topic.")

        if self.publish_rate <= 0.0:
            self.publish_rate = 30.0
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        # Allow runtime param updates
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"Xela AH JSP ready: output={self.output_topic}, rate={self.publish_rate}Hz, "
            f"keep_joints={self.keep_joints or '(all non-fixed from URDF)'}"
        )

    # ----- Helpers -----
    def _get_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _parse_initial_positions(self, param_val):
        if isinstance(param_val, str):
            try:
                return {str(k): float(v) for k, v in json.loads(param_val).items()}
            except Exception:
                return {}
        if isinstance(param_val, dict):
            return {str(k): float(v) for k, v in param_val.items()}
        return {}

    # ----- YAML config -----
    def _load_config_yaml(self, initial=False):
        path = self.config_yaml.strip()
        if not path:
            if initial:
                self.get_logger().info("No config_yaml provided; using parameters as-is.")
            return
        p = Path(path)
        if not p.exists():
            self.get_logger().error(f"config_yaml not found: {p}")
            return
        try:
            with p.open("r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to read config_yaml: {e}")
            return

        if "keep_joints" in cfg and isinstance(cfg["keep_joints"], list):
            self.keep_joints = [str(x) for x in cfg["keep_joints"]]
            self.get_logger().info(f"[YAML] keep_joints loaded: {self.keep_joints}")

        if "ordered_keep_joints" in cfg and isinstance(cfg["ordered_keep_joints"], list):
            self.ordered_keep_joints = [str(x) for x in cfg["ordered_keep_joints"]]

        if "initial_positions" in cfg and isinstance(cfg["initial_positions"], dict):
            self.initial_positions = {str(k): float(v) for k, v in cfg["initial_positions"].items()}

        if "publish_rate" in cfg:
            try:
                pr = float(cfg["publish_rate"])
                if pr > 0.0:
                    self.publish_rate = pr
                    # Timer may not exist yet during __init__
                    if hasattr(self, "timer") and self.timer is not None:
                        self.timer.cancel()
                        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)
            except Exception:
                pass

        if "output_topic" in cfg:
            self.output_topic = str(cfg["output_topic"])
            self.pub = self.create_publisher(JointState, self.output_topic, 10)

    # ----- URDF/XACRO loading -----
    def _on_robot_description(self, msg: StringMsg):
        self._load_urdf(msg.data)

    def _load_urdf(self, xml_or_path: str):
        if URDF is None:
            self.get_logger().error("URDF parser not available. Install 'ros-humble-urdf' and 'ros-humble-urdfdom-py'.")
            return

        s = (xml_or_path or "").lstrip()   # 앞 공백 제거
        xml_str = None

        # 1) 먼저 'XML인지' 판별: <?xml...> 또는 <robot ...> 같은 시그니처를 앞부분에서 확인
        looks_like_xml = s.startswith("<?xml") or (s.startswith("<") and "<robot" in s[:500])
        if looks_like_xml:
            xml_str = s
        else:
            # 2) 파일 경로로 취급 (XML이 아닌 경우에만 OS stat 호출)
            p = Path(s)
            try:
                if p.exists() and p.is_file():
                    if p.suffix.lower() == ".xacro":
                        if xacro is None:
                            self.get_logger().error("XACRO requested but not available. Install 'ros-humble-xacro'.")
                            return
                        try:
                            doc = xacro.process_file(str(p))
                            xml_str = doc.toxml()
                            self.get_logger().info(f"Loaded XACRO: {p}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to process XACRO '{p}': {e}")
                            return
                    else:
                        try:
                            xml_str = p.read_text(encoding="utf-8")
                            self.get_logger().info(f"Loaded URDF file: {p}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to read URDF file '{p}': {e}")
                            return
                else:
                    # 파일도 아니고 XML도 아니면, 토픽/파라미터에서 온 'XML 문자열'일 가능성이 큼 → XML로 간주
                    self.get_logger().warn("URDF input does not look like a file; treating input as an XML string.")
                    xml_str = s
            except Exception as e:
                # 매우 긴 문자열을 경로로 stat 하다가 생길 수 있는 OSError(Errno 36)도 여기서 흡수
                self.get_logger().warn(f"Path check failed ({e}); treating input as an XML string.")
                xml_str = s

        # 3) URDF 파싱
        try:
            model = URDF.from_xml_string(xml_str)
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return

        nonfixed = []
        for j in model.joints:
            try:
                if j.type in NON_FIXED_TYPES:
                    nonfixed.append(j.name)
            except Exception:
                pass

        with self._urdf_lock:
            self._known_nonfixed_joints = nonfixed
            self._has_urdf = True

        self.get_logger().info(f"URDF ready. Non-fixed joints: {len(nonfixed)}")

    # ----- JointState ingestion & publish -----
    def _on_joint_state(self, msg: JointState):
        if not msg.name:
            return
        self._last_input_order = list(msg.name)
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        def pick(arr, n):
            if arr and len(arr) == len(msg.name):
                return arr[name_to_idx[n]]
            return None

        for n in msg.name:
            cur = self._last_js_map.get(n, {})
            pos = pick(msg.position, n)
            vel = pick(msg.velocity, n)
            eff = pick(msg.effort,   n)
            if pos is not None:
                cur["pos"] = float(pos)
            if vel is not None:
                cur["vel"] = float(vel)
            if eff is not None:
                cur["eff"] = float(eff)
            self._last_js_map[n] = cur

    def _on_timer(self):
        with self._urdf_lock:
            base_list = list(self.keep_joints) if self.keep_joints else list(self._known_nonfixed_joints)

        if not base_list:
            return

        if self.ordered_keep_joints:
            out_names = [n for n in self.ordered_keep_joints if n in base_list]
        elif self.preserve_input_order and self._last_input_order:
            out_names = [n for n in self._last_input_order if n in base_list]
            for n in base_list:
                if n not in out_names:
                    out_names.append(n)
        else:
            out_names = base_list

        if not out_names:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = out_names

        positions, velocities, efforts = [], [], []
        has_vel, has_eff = False, False

        for n in out_names:
            src = self._last_js_map.get(n, {})
            positions.append(float(src.get("pos", self.initial_positions.get(n, 0.0))))
            v = src.get("vel", None)
            e = src.get("eff", None)
            if v is not None:
                velocities.append(float(v))
                has_vel = True
            else:
                velocities.append(0.0)
            if e is not None:
                efforts.append(float(e))
                has_eff = True
            else:
                efforts.append(0.0)

        js.position = positions
        if has_vel:
            js.velocity = velocities
        if has_eff:
            js.effort = efforts

        self.pub.publish(js)

    # ----- Dynamic params -----
    def _on_set_params(self, params):
        for p in params:
            if p.name == "keep_joints":
                self.keep_joints = list(p.value.string_array_value)
            elif p.name == "ordered_keep_joints":
                self.ordered_keep_joints = list(p.value.string_array_value)
            elif p.name == "publish_rate":
                new_rate = float(p.value.double_value)
                if new_rate > 0.0:
                    self.publish_rate = new_rate
                    if hasattr(self, "timer") and self.timer is not None:
                        self.timer.cancel()
                        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)
            elif p.name == "output_topic":
                self.output_topic = p.value.string_value
                self.pub = self.create_publisher(JointState, self.output_topic, 10)
            # elif p.name == "initial_positions":
            #     try:
            #         if isinstance(p.value.string_value, str) and p.value.string_value:
            #             self.initial_positions = json.loads(p.value.string_value)
            #     except Exception:
            #         pass
            elif p.name == "initial_positions":
                try:
                    raw = p.value.string_value
                    if raw:
                        try:
                            parsed = json.loads(raw)
                        except Exception:
                            parsed = yaml.safe_load(raw)
                        if isinstance(parsed, dict):
                            self.initial_positions = {str(k): float(v) for k, v in parsed.items()}
                        else:
                            self.get_logger().warn("initial_positions must be a dict JSON/YAML string; ignoring.")
                    else:
                        self.initial_positions = {}
                except Exception as e:
                    self.get_logger().warn(f"Failed to parse initial_positions: {e}")

            elif p.name == "config_yaml":
                self.config_yaml = p.value.string_value
                self._load_config_yaml(initial=False)
            elif p.name == "robot_description_from":
                self.robot_description_from = p.value.string_value
            elif p.name == "urdf_file":
                self.urdf_file = p.value.string_value
                if self.robot_description_from == "file" and self.urdf_file:
                    self._load_urdf(self.urdf_file)
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = XelaAhJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
