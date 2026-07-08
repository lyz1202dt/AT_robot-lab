import copy
import math
import os
import threading
from datetime import datetime
from pathlib import Path
from tkinter import BOTH, END, LEFT, RIGHT, Button, Frame, Label, Listbox, Tk

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_msgs.msg import Cmd
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class ObstacleGameUiNode(Node):
    def __init__(self):
        super().__init__("obstacle_game_ui_node")
        self.current_policy_id = None
        self.create_subscription(Cmd, "robot_move_cmd", self._cmd_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _cmd_callback(self, msg):
        self.current_policy_id = int(msg.mode)

    def get_policy_id(self):
        if self.current_policy_id is None:
            return 2
        return self.current_policy_id

    def get_base_link_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Failed to lookup map->base_link: {exc}")
            return 0.0, 0.0, 0.0

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), float(yaw)


class ObstacleGameUi:
    def __init__(self, node):
        self.node = node
        self.config_dir = self._find_config_dir()
        self.output_dir = self._find_workspace_root()
        self.configs = self._load_configs()
        self.paths = []
        self.recording = False
        self.record_file = None
        self.record_path = None

        self.root = Tk()
        self.root.title("Obstacle Game UI")
        self.root.geometry("720x480")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.main_frame = Frame(self.root, padx=12, pady=12)
        self.main_frame.pack(fill=BOTH, expand=True)

        self.record_button = Button(
            self.main_frame,
            text="开始录制轨迹点",
            height=3,
            font=("Arial", 18),
            command=self.toggle_recording,
        )
        self.record_button.pack(fill="x")

        self.record_controls = None
        self.record_point_button = None
        self.delete_point_button = None
        self.config_list = None
        self.config_label = None

    def _find_config_dir(self):
        source_config_dir = Path(__file__).resolve().parents[1] / "config"
        if source_config_dir.exists():
            return source_config_dir
        return Path(get_package_share_directory("obstacle_game_ui")) / "config"

    def _find_workspace_root(self):
        current = Path.cwd().resolve()
        for path in (current, *current.parents):
            if (path / "src" / "obstacle_game_ui").exists():
                return path
        return current

    def _load_configs(self):
        configs = {}
        for path in sorted(self.config_dir.glob("*.yaml")):
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            default_config = data.get("config", [{}])
            if isinstance(default_config, list):
                default_config = default_config[0] if default_config else {}
            configs[path.name] = default_config or {}
        return configs

    def run(self):
        self.root.mainloop()

    def close(self):
        if self.recording:
            self.stop_recording()
        self.root.destroy()

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        self.recording = True
        self.paths = []
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.record_path = self.output_dir / f".recording_{timestamp}.yaml"
        self.record_file = self.record_path.open("w", encoding="utf-8")
        self._write_record_file()

        self.record_button.configure(text="停止录制轨迹点")
        self._create_record_controls()

    def stop_recording(self):
        self._write_record_file()
        self.record_file.close()
        self.record_file = None

        final_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        final_path = self.output_dir / f"record{final_timestamp}.yaml"
        os.replace(self.record_path, final_path)
        self.node.get_logger().info(f"Saved trajectory points to {final_path}")
        self.record_path = None

        self.recording = False
        self.record_button.configure(text="开始录制轨迹点")
        self._destroy_record_controls()

    def _create_record_controls(self):
        self.record_controls = Frame(self.main_frame, pady=12)
        self.record_controls.pack(fill=BOTH, expand=True)

        button_frame = Frame(self.record_controls)
        button_frame.pack(fill="x")

        self.record_point_button = Button(
            button_frame,
            text="记录点位信息",
            height=2,
            command=self.record_point,
        )
        self.record_point_button.pack(
            side=LEFT,
            fill="x",
            expand=True,
            padx=(0, 6),
        )

        self.delete_point_button = Button(
            button_frame,
            text="删除上一个点位",
            height=2,
            command=self.delete_last_point,
        )
        self.delete_point_button.pack(
            side=RIGHT,
            fill="x",
            expand=True,
            padx=(6, 0),
        )

        content_frame = Frame(self.record_controls, pady=12)
        content_frame.pack(fill=BOTH, expand=True)

        self.config_list = Listbox(
            content_frame,
            exportselection=False,
            width=24,
        )
        self.config_list.pack(side=LEFT, fill="y")
        self.config_list.bind("<<ListboxSelect>>", self.update_config_label)

        for name in self.configs:
            self.config_list.insert(END, name)

        self.config_label = Label(
            content_frame,
            text="",
            justify=LEFT,
            anchor="nw",
            padx=12,
            font=("Monospace", 10),
        )
        self.config_label.pack(side=LEFT, fill=BOTH, expand=True)

        if self.configs:
            self.config_list.selection_set(0)
            self.update_config_label()

    def _destroy_record_controls(self):
        if self.record_controls is not None:
            self.record_controls.destroy()
        self.record_controls = None
        self.record_point_button = None
        self.delete_point_button = None
        self.config_list = None
        self.config_label = None

    def selected_config_name(self):
        if not self.config_list:
            return None
        selection = self.config_list.curselection()
        if not selection:
            return None
        return self.config_list.get(selection[0])

    def selected_config(self):
        name = self.selected_config_name()
        if name is None:
            return {}
        return copy.deepcopy(self.configs.get(name, {}))

    def update_config_label(self, _event=None):
        if self.config_label is None:
            return
        name = self.selected_config_name()
        if name is None:
            self.config_label.configure(text="")
            return
        text = yaml.safe_dump(
            self.configs.get(name, {}),
            allow_unicode=True,
            sort_keys=False,
        )
        self.config_label.configure(text=text)

    def record_point(self):
        x, y, yaw = self.node.get_base_link_pose()
        point = {
            "policy_id": self.node.get_policy_id(),
            "target_pos": {
                "x": round(x, 6),
                "y": round(y, 6),
            },
            "target_yaw": round(yaw, 6),
        }
        point.update(self.selected_config())
        self.paths.append(point)
        self._write_record_file()
        self.node.get_logger().info(f"Recorded point #{len(self.paths)}")

    def delete_last_point(self):
        if not self.paths:
            return
        self.paths.pop()
        self._write_record_file()
        remaining = len(self.paths)
        self.node.get_logger().info(
            f"Deleted last point, {remaining} remaining"
        )

    def _write_record_file(self):
        if self.record_file is None:
            return
        self.record_file.seek(0)
        yaml.safe_dump(
            {"paths": self.paths},
            self.record_file,
            allow_unicode=True,
            sort_keys=False,
        )
        self.record_file.truncate()
        self.record_file.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGameUiNode()

    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(
        target=spin_node,
        daemon=True,
    )
    spin_thread.start()

    ui = ObstacleGameUi(node)
    try:
        ui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
