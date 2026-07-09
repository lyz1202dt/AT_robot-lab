#!/usr/bin/env python3

import queue
import subprocess
import sys
import threading
import time
from typing import Iterable, List, Optional, Sequence

try:
    import tkinter as tk
except ImportError as exc:
    print("task_game_ui requires python3-tk / tkinter.", file=sys.stderr)
    raise exc

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from robot_msgs.msg import Remote
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension


ROWS = 2
COLS = 4
GRID_SIZE = ROWS * COLS
START_GAME_PARAM_NODE = "robot_calc_node"
START_GAME_PARAM = "start_game"
START_CALC_PARAM_NODE = "arithmetic_node"
START_CALC_PARAM = "start_calc"
CALC_TEST_PARAM = "show_image"
RETRY_PARAM_NODE = "robot_calc"
RETRY_PARAM = "is_first_game"
RETRY_SCRIPT = "./AAA.sh"
MINUTE_PREPARE_REMOTE_KEY = 16
AUTO_REMOTE_KEY = 2
MANUAL_REMOTE_KEY = 0
FORCE_STOP_NODES = ("rl_real_atdog2", "rl_real_atdog3", "robot_control")

COLOR_SEQUENCE = (255, 0, 1, 2, 3)
COLOR_STYLES = {
    255: {"bg": "#ffffff", "fg": "#111827", "active_bg": "#f3f4f6"},
    3: {"bg": "#22c55e", "fg": "#052e16", "active_bg": "#16a34a"},
    2: {"bg": "#9ca3af", "fg": "#111827", "active_bg": "#6b7280"},
    0: {"bg": "#2563eb", "fg": "#ffffff", "active_bg": "#1d4ed8"},
    1: {"bg": "#dc2626", "fg": "#ffffff", "active_bg": "#b91c1c"},
}


Grid = List[List[int]]


def default_grid() -> Grid:
    return [[255 for _ in range(COLS)] for _ in range(ROWS)]


def flatten_grid(grid: Sequence[Sequence[int]]) -> List[int]:
    return [int(grid[row][col]) for row in range(ROWS) for col in range(COLS)]


def normalize_value(value: int) -> int:
    value = int(value)
    return value if value in COLOR_STYLES else 255


def normalize_grid(values: Iterable[int]) -> Grid:
    flat = [normalize_value(value) for value in list(values)[:GRID_SIZE]]
    if len(flat) < GRID_SIZE:
        flat.extend([255] * (GRID_SIZE - len(flat)))
    return [flat[row * COLS : (row + 1) * COLS] for row in range(ROWS)]


def next_color_value(value: int) -> int:
    value = normalize_value(value)
    index = COLOR_SEQUENCE.index(value)
    return COLOR_SEQUENCE[(index + 1) % len(COLOR_SEQUENCE)]


def make_grid_message(grid: Sequence[Sequence[int]]) -> Int32MultiArray:
    msg = Int32MultiArray()

    row_dim = MultiArrayDimension()
    row_dim.label = "rows"
    row_dim.size = ROWS
    row_dim.stride = GRID_SIZE

    col_dim = MultiArrayDimension()
    col_dim.label = "cols"
    col_dim.size = COLS
    col_dim.stride = COLS

    msg.layout.dim = [row_dim, col_dim]
    msg.layout.data_offset = 0
    msg.data = flatten_grid(grid)
    return msg


class BoxIdGridNode(Node):
    def __init__(
        self,
        ui_updates: "queue.Queue[Grid]",
        vip_box_id_updates: "queue.Queue[int]",
    ) -> None:
        super().__init__("task_game_ui")
        self._ui_updates = ui_updates
        self._vip_box_id_updates = vip_box_id_updates
        self._publisher = self.create_publisher(Int32MultiArray, "box_id_grid", 10)
        self._remote_publisher = self.create_publisher(Remote, "remote", 10)
        self._start_game_client = self.create_client(
            SetParameters,
            f"/{START_GAME_PARAM_NODE}/set_parameters",
        )
        self._start_calc_client = self.create_client(
            SetParameters,
            f"/{START_CALC_PARAM_NODE}/set_parameters",
        )
        self._retry_param_client = self.create_client(
            SetParameters,
            f"/{RETRY_PARAM_NODE}/set_parameters",
        )
        self._subscription = self.create_subscription(
            Int32MultiArray,
            "box_id_grid",
            self._on_box_id_grid,
            10,
        )
        self._vip_box_id_subscription = self.create_subscription(
            Int32,
            "vip_box_id",
            self._on_vip_box_id,
            10,
        )

    def publish_grid(self, grid: Sequence[Sequence[int]]) -> None:
        msg = make_grid_message(grid)
        self._publisher.publish(msg)
        self.get_logger().info(f"Published box_id_grid: {list(msg.data)}")

    def set_start_game(self) -> None:
        self._set_bool_parameter(
            self._start_game_client,
            START_GAME_PARAM_NODE,
            START_GAME_PARAM,
            True,
        )
        self._set_bool_parameter(
            self._start_calc_client,
            START_CALC_PARAM_NODE,
            START_CALC_PARAM,
            True,
        )

    def minute_prepare(self) -> None:
        self._set_bool_parameter(
            self._start_calc_client,
            START_CALC_PARAM_NODE,
            CALC_TEST_PARAM,
            True,
        )
        self._publish_remote_command(MINUTE_PREPARE_REMOTE_KEY)

    def switch_to_auto(self) -> None:
        self._set_bool_parameter(
            self._start_calc_client,
            START_CALC_PARAM_NODE,
            CALC_TEST_PARAM,
            False,
        )
        self._publish_remote_command(AUTO_REMOTE_KEY)

    def switch_to_manual(self) -> None:
        self._publish_remote_command(MANUAL_REMOTE_KEY)

    def emergency_stop(self) -> None:
        for node_name in FORCE_STOP_NODES:
            self._force_stop_node(node_name)

    def retry_this_run(self) -> None:
        thread = threading.Thread(target=self._retry_this_run_worker, daemon=True)
        thread.start()

    def _retry_this_run_worker(self) -> None:
        try:
            process = subprocess.Popen(["bash", RETRY_SCRIPT])
        except OSError as exc:
            self.get_logger().error(f"Failed to run {RETRY_SCRIPT}: {exc}")
            return

        self.get_logger().info(f"Started {RETRY_SCRIPT}, waiting 5 seconds")
        time.sleep(5.0)
        return_code = process.poll()
        if return_code is not None and return_code != 0:
            self.get_logger().error(
                f"{RETRY_SCRIPT} exited with code {return_code}, skip retry parameter"
            )
            return

        self._set_bool_parameter(
            self._retry_param_client,
            RETRY_PARAM_NODE,
            RETRY_PARAM,
            False,
            timeout_sec=5.0,
        )

    def _force_stop_node(self, node_name: str) -> None:
        # rl_real_atdog2/3、robot_control 在代码里的 rclcpp 节点名（robot_controller_node、
        # robot_calc_node）不会出现在进程命令行里，只能按可执行文件名匹配进程。
        detect = subprocess.run(
            ["pgrep", "-f", f"/{node_name}$"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if detect.returncode != 0:
            detect = subprocess.run(
                ["pgrep", "-x", node_name],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        if detect.returncode != 0:
            self.get_logger().warn(f"No running process matched node: {node_name}")
            return

        stopped = False
        for command in (
            ["pkill", "-9", "-f", f"/{node_name}$"],
            ["pkill", "-9", "-x", node_name],
        ):
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            stopped = stopped or result.returncode == 0

        if stopped:
            self.get_logger().warn(f"Force stopped ROS node process: {node_name}")
        else:
            self.get_logger().error(f"Failed to stop ROS node process: {node_name}")

    def _publish_remote_command(self, key: int) -> None:
        msg = Remote()
        msg.lx = 0.0
        msg.ly = 0.0
        msg.rx = 0.0
        msg.ry = 0.0
        msg.key = key
        msg.just_reconnected = False
        self._remote_publisher.publish(msg)
        self.get_logger().info(f"Published remote command: key={key}")

    def _set_bool_parameter(
        self,
        client,
        node_name: str,
        param_name: str,
        value: bool,
        timeout_sec: float = 0.0,
    ) -> None:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Parameter service /{node_name}/set_parameters is not available"
            )
            return

        parameter = Parameter()
        parameter.name = param_name
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=value,
        )

        request = SetParameters.Request()
        request.parameters = [parameter]
        future = client.call_async(request)
        future.add_done_callback(
            lambda done_future: self._on_bool_parameter_set(
                done_future,
                node_name,
                param_name,
                value,
            )
        )

    def _on_bool_parameter_set(
        self,
        future,
        node_name: str,
        param_name: str,
        value: bool,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to set {node_name}.{param_name}: {exc}")
            return

        if not response.results:
            self.get_logger().error(f"Failed to set {node_name}.{param_name}: empty result")
            return

        result = response.results[0]
        if result.successful:
            self.get_logger().info(f"Set {node_name}.{param_name}={str(value).lower()}")
        else:
            reason = result.reason or "no reason provided"
            self.get_logger().error(
                f"Failed to set {node_name}.{param_name}: {reason}"
            )

    def _on_box_id_grid(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < GRID_SIZE:
            self.get_logger().error(
                f"box_id_grid needs at least {GRID_SIZE} values, got {len(msg.data)}"
            )
            return
        self._ui_updates.put(normalize_grid(msg.data))

    def _on_vip_box_id(self, msg: Int32) -> None:
        self._vip_box_id_updates.put(int(msg.data))


class TaskGameUi:
    def __init__(
        self,
        root: tk.Tk,
        ros_node: BoxIdGridNode,
        ui_updates: "queue.Queue[Grid]",
        vip_box_id_updates: "queue.Queue[int]",
    ) -> None:
        self._root = root
        self._ros_node = ros_node
        self._ui_updates = ui_updates
        self._vip_box_id_updates = vip_box_id_updates
        self._grid = default_grid()
        self._vip_box_id: Optional[int] = None
        self._buttons: List[List[tk.Button]] = []
        self._vip_box_id_label: Optional[tk.Label] = None
        self._after_id = None

        self._build_window()
        self._refresh_all_buttons()
        self._refresh_vip_box_id()
        self._after_id = self._root.after(50, self._poll_ros_updates)

    def _build_window(self) -> None:
        self._root.title("Task Game UI")
        screen_width = self._root.winfo_screenwidth()
        screen_height = self._root.winfo_screenheight()
        compact = screen_height <= 720

        self._shell_pad = 12 if compact else 22
        self._action_gap = 8 if compact else 14
        self._button_font = ("Sans", 18 if compact else 22, "bold")
        self._grid_font = ("Sans", 16 if compact else 18, "bold")
        self._emergency_font = ("Sans", 24 if compact else 28, "bold")
        self._vip_font = ("Sans", 48 if compact else 72, "bold")
        self._primary_button_height = 1 if compact else 2
        self._emergency_button_height = 4 if compact else 5

        min_width = max(560, min(820, screen_width - 80))
        min_height = max(420, min(540 if compact else 600, screen_height - 90))
        self._root.minsize(min_width, min_height)
        self._root.configure(bg="#f5f7fb")
        self._root.grid_columnconfigure(0, weight=1)
        self._root.grid_rowconfigure(0, weight=1)

        shell = tk.Frame(self._root, bg="#f5f7fb", padx=self._shell_pad, pady=self._shell_pad)
        shell.grid(row=0, column=0, sticky="nsew")
        shell.grid_columnconfigure(0, weight=1)
        shell.grid_columnconfigure(1, weight=0)
        shell.grid_rowconfigure(0, weight=1)

        left_frame = tk.Frame(shell, bg="#f5f7fb")
        left_frame.grid(row=0, column=0, sticky="nsew")
        left_frame.grid_columnconfigure(0, weight=1)
        left_frame.grid_rowconfigure(1, weight=1, minsize=150 if compact else 190)

        right_frame = tk.Frame(shell, bg="#f5f7fb")
        right_frame.grid(row=0, column=1, sticky="ns", padx=(self._shell_pad, 0))
        right_frame.grid_columnconfigure(0, weight=1, minsize=160 if compact else 200)
        right_frame.grid_rowconfigure(0, weight=1)

        action_frame = tk.Frame(left_frame, bg="#f5f7fb")
        action_frame.grid(row=0, column=0, sticky="ew", pady=(0, self._action_gap))
        action_frame.grid_columnconfigure(0, weight=1, uniform="actions")
        action_frame.grid_columnconfigure(1, weight=1, uniform="actions")

        minute_prepare_button = self._create_primary_button(
            action_frame,
            "一分钟准备",
            self._minute_prepare,
        )
        minute_prepare_button.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, self._action_gap // 2),
            pady=(0, self._action_gap),
        )

        auto_button = self._create_primary_button(
            action_frame,
            "切入自动",
            self._switch_to_auto,
        )
        auto_button.grid(row=1, column=0, sticky="ew", padx=(0, self._action_gap // 2))

        start_button = self._create_primary_button(
            action_frame,
            "比赛开始",
            self._start_game,
        )
        start_button.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(self._action_gap // 2, 0),
            pady=(0, self._action_gap),
        )

        confirm_button = self._create_primary_button(action_frame, "确定", self._publish_grid)
        confirm_button.grid(row=1, column=1, sticky="ew", padx=(self._action_gap // 2, 0))

        emergency_stop_button = self._create_emergency_button(
            right_frame,
            "紧急关停",
            self._emergency_stop,
        )
        emergency_stop_button.grid(row=0, column=0, sticky="nsew", pady=(0, self._action_gap))

        manual_button = self._create_primary_button(
            right_frame,
            "切入手动",
            self._switch_to_manual,
        )
        manual_button.grid(row=1, column=0, sticky="ew", pady=(0, self._action_gap))

        retry_button = self._create_primary_button(
            right_frame,
            "本次重试",
            self._retry_this_run,
        )
        retry_button.grid(row=2, column=0, sticky="ew")

        grid_frame = tk.Frame(left_frame, bg="#f5f7fb")
        grid_frame.grid(row=1, column=0, sticky="nsew")
        for row in range(ROWS):
            grid_frame.grid_rowconfigure(row, weight=1, uniform="grid_rows")
        for col in range(COLS):
            grid_frame.grid_columnconfigure(col, weight=1, uniform="grid_cols")

        for row in range(ROWS):
            button_row: List[tk.Button] = []
            for col in range(COLS):
                button = tk.Button(
                    grid_frame,
                    command=lambda r=row, c=col: self._advance_button(r, c),
                    font=self._grid_font,
                    relief=tk.FLAT,
                    borderwidth=0,
                    cursor="hand2",
                )
                button.grid(
                    row=row,
                    column=col,
                    sticky="nsew",
                    padx=self._action_gap // 2,
                    pady=self._action_gap // 2,
                )
                button_row.append(button)
            self._buttons.append(button_row)

        self._vip_box_id_label = tk.Label(
            left_frame,
            bg="#f5f7fb",
            fg="#111827",
            font=self._vip_font,
            anchor="center",
        )
        self._vip_box_id_label.grid(row=2, column=0, sticky="ew", pady=(self._action_gap, 0))

    def _create_primary_button(self, parent: tk.Widget, text: str, command) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg="#111827",
            fg="#ffffff",
            activebackground="#374151",
            activeforeground="#ffffff",
            font=self._button_font,
            relief=tk.FLAT,
            borderwidth=0,
            height=self._primary_button_height,
            cursor="hand2",
        )

    def _create_emergency_button(self, parent: tk.Widget, text: str, command) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg="#dc2626",
            fg="#ffffff",
            activebackground="#991b1b",
            activeforeground="#ffffff",
            font=self._emergency_font,
            relief=tk.FLAT,
            borderwidth=0,
            width=8,
            height=self._emergency_button_height,
            cursor="hand2",
        )

    def _advance_button(self, row: int, col: int) -> None:
        self._grid[row][col] = next_color_value(self._grid[row][col])
        self._refresh_button(row, col)

    def _start_game(self) -> None:
        self._ros_node.set_start_game()

    def _minute_prepare(self) -> None:
        self._ros_node.minute_prepare()

    def _switch_to_auto(self) -> None:
        self._ros_node.switch_to_auto()

    def _switch_to_manual(self) -> None:
        self._ros_node.switch_to_manual()

    def _emergency_stop(self) -> None:
        self._ros_node.emergency_stop()

    def _retry_this_run(self) -> None:
        self._ros_node.retry_this_run()

    def _publish_grid(self) -> None:
        snapshot = [row[:] for row in self._grid]
        self._ros_node.publish_grid(snapshot)

    def _poll_ros_updates(self) -> None:
        grid_updated = False
        vip_box_id_updated = False
        while True:
            try:
                self._grid = self._ui_updates.get_nowait()
                grid_updated = True
            except queue.Empty:
                break

        while True:
            try:
                self._vip_box_id = self._vip_box_id_updates.get_nowait()
                vip_box_id_updated = True
            except queue.Empty:
                break

        if grid_updated:
            self._refresh_all_buttons()

        if vip_box_id_updated:
            self._refresh_vip_box_id()

        self._after_id = self._root.after(50, self._poll_ros_updates)

    def _refresh_all_buttons(self) -> None:
        for row in range(ROWS):
            for col in range(COLS):
                self._refresh_button(row, col)

    def _refresh_button(self, row: int, col: int) -> None:
        value = normalize_value(self._grid[row][col])
        self._grid[row][col] = value
        style = COLOR_STYLES[value]
        self._buttons[row][col].configure(
            text=str(value),
            bg=style["bg"],
            fg=style["fg"],
            activebackground=style["active_bg"],
            activeforeground=style["fg"],
        )

    def _refresh_vip_box_id(self) -> None:
        if self._vip_box_id_label is None:
            return
        text = "" if self._vip_box_id is None else str(self._vip_box_id)
        self._vip_box_id_label.configure(text=text)

    def close(self) -> None:
        if self._after_id is not None:
            try:
                self._root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    ui_updates: "queue.Queue[Grid]" = queue.Queue()
    vip_box_id_updates: "queue.Queue[int]" = queue.Queue()
    ros_node = BoxIdGridNode(ui_updates, vip_box_id_updates)
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    app = None
    try:
        root = tk.Tk()
        app = TaskGameUi(root, ros_node, ui_updates, vip_box_id_updates)

        def on_close() -> None:
            if app is not None:
                app.close()
            root.destroy()

        root.protocol("WM_DELETE_WINDOW", on_close)
        root.mainloop()
    except tk.TclError as exc:
        ros_node.get_logger().error(f"Failed to start tkinter UI: {exc}")
        raise SystemExit(1) from exc
    finally:
        if app is not None:
            app.close()
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
