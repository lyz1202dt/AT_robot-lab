#!/usr/bin/env python3

import queue
import sys
import threading
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
CALC_TEST_PARAM = "test"
MINUTE_PREPARE_REMOTE_KEY = 2 + 16
AUTO_REMOTE_KEY = 1

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
    ) -> None:
        if not client.wait_for_service(timeout_sec=0.0):
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
        self._root.minsize(560, 360)
        self._root.configure(bg="#f5f7fb")
        self._root.grid_columnconfigure(0, weight=1)
        self._root.grid_rowconfigure(0, weight=1)

        shell = tk.Frame(self._root, bg="#f5f7fb", padx=22, pady=22)
        shell.grid(row=0, column=0, sticky="nsew")
        shell.grid_columnconfigure(0, weight=1)
        shell.grid_rowconfigure(4, weight=1)

        minute_prepare_button = self._create_primary_button(
            shell,
            "一分钟准备",
            self._minute_prepare,
        )
        minute_prepare_button.grid(row=0, column=0, sticky="ew", pady=(0, 12))

        auto_button = self._create_primary_button(
            shell,
            "切入自动",
            self._switch_to_auto,
        )
        auto_button.grid(row=1, column=0, sticky="ew", pady=(0, 12))

        start_button = self._create_primary_button(
            shell,
            "比赛开始",
            self._start_game,
        )
        start_button.grid(row=2, column=0, sticky="ew", pady=(0, 12))

        confirm_button = self._create_primary_button(shell, "确定", self._publish_grid)
        confirm_button.grid(row=3, column=0, sticky="ew", pady=(0, 18))

        grid_frame = tk.Frame(shell, bg="#f5f7fb")
        grid_frame.grid(row=4, column=0, sticky="nsew")
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
                    font=("Sans", 18, "bold"),
                    relief=tk.FLAT,
                    borderwidth=0,
                    cursor="hand2",
                )
                button.grid(row=row, column=col, sticky="nsew", padx=7, pady=7)
                button_row.append(button)
            self._buttons.append(button_row)

        self._vip_box_id_label = tk.Label(
            shell,
            bg="#f5f7fb",
            fg="#111827",
            font=("Sans", 72, "bold"),
            anchor="center",
        )
        self._vip_box_id_label.grid(row=5, column=0, sticky="ew", pady=(18, 0))

    def _create_primary_button(self, parent: tk.Widget, text: str, command) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg="#111827",
            fg="#ffffff",
            activebackground="#374151",
            activeforeground="#ffffff",
            font=("Sans", 22, "bold"),
            relief=tk.FLAT,
            borderwidth=0,
            height=2,
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
