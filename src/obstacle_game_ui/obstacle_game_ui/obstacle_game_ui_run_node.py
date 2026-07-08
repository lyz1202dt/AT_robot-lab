import threading
from functools import partial
from tkinter import BOTH, LEFT, RIGHT, Y, Button, Frame, Label, Tk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ObstacleGameUiRunNode(Node):
    def __init__(self):
        super().__init__("obstacle_game_ui_run_node")
        self.declare_parameter("target_node", "robot_calc_node")
        target_node = self.get_parameter("target_node").value
        service_name = self._parameter_service_name(target_node)
        self.set_parameters_client = self.create_client(
            SetParameters,
            service_name,
        )
        self.get_logger().info(f"Using parameter service: {service_name}")

    @staticmethod
    def _parameter_service_name(node_name):
        normalized = str(node_name).strip("/")
        return f"/{normalized}/set_parameters"

    def _wait_for_parameter_service(self, done_callback):
        if not self.set_parameters_client.service_is_ready():
            self.get_logger().warn("Waiting for robot_calc_node parameter service")

        if not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            message = "robot_calc_node parameter service is not available"
            self.get_logger().error(message)
            done_callback(False, message)
            return False

        return True

    def set_switch_path(self, path_id, done_callback):
        if not self._wait_for_parameter_service(done_callback):
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="switch_path",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=int(path_id),
                ),
            ),
        ]
        future = self.set_parameters_client.call_async(request)
        future.add_done_callback(
            partial(self._handle_set_parameters_result, path_id, done_callback)
        )

    def set_target_trigger(self, parameter_name, display_name, done_callback):
        if not self._wait_for_parameter_service(done_callback):
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=parameter_name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL,
                    bool_value=True,
                ),
            ),
        ]
        future = self.set_parameters_client.call_async(request)
        future.add_done_callback(
            partial(
                self._handle_trigger_result,
                parameter_name,
                display_name,
                done_callback,
            )
        )

    def _handle_set_parameters_result(self, path_id, done_callback, future):
        try:
            response = future.result()
        except Exception as exc:
            message = f"Failed to set switch_path={path_id}: {exc}"
            self.get_logger().error(message)
            done_callback(False, message)
            return

        if not response.results:
            message = f"Set switch_path={path_id} returned no result"
            self.get_logger().error(message)
            done_callback(False, message)
            return

        result = response.results[0]
        if result.successful:
            message = f"switch_path set to {path_id}"
            self.get_logger().info(message)
            done_callback(True, message)
        else:
            reason = result.reason or "unknown reason"
            message = f"Failed to set switch_path={path_id}: {reason}"
            self.get_logger().error(message)
            done_callback(False, message)

    def _handle_trigger_result(
        self,
        parameter_name,
        display_name,
        done_callback,
        future,
    ):
        try:
            response = future.result()
        except Exception as exc:
            message = f"Failed to set {parameter_name}=true: {exc}"
            self.get_logger().error(message)
            done_callback(False, message)
            return

        if not response.results:
            message = f"Set {parameter_name}=true returned no result"
            self.get_logger().error(message)
            done_callback(False, message)
            return

        result = response.results[0]
        if result.successful:
            message = f"{display_name}指令已发送"
            self.get_logger().info(message)
            done_callback(True, message)
        else:
            reason = result.reason or "unknown reason"
            message = f"Failed to set {parameter_name}=true: {reason}"
            self.get_logger().error(message)
            done_callback(False, message)


class ObstacleGameUiRun:
    def __init__(self, node):
        self.node = node
        self.root = Tk()
        self.root.title("Obstacle Game Run")
        self.root.geometry("620x450")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.main_frame = Frame(self.root, padx=18, pady=18)
        self.main_frame.pack(fill=BOTH, expand=True)

        self.status_label = Label(
            self.main_frame,
            text="请选择路径",
            anchor="center",
            font=("Arial", 14),
        )
        self.status_label.pack(fill="x", pady=(0, 14))

        self.button_frame = Frame(self.main_frame)
        self.button_frame.pack(fill=BOTH, expand=True)

        self.path_frame = Frame(self.button_frame)
        self.path_frame.pack(side=LEFT, fill=BOTH, expand=True)

        for text, path_id in (
            ("绕过台阶", 1),
            ("绕过桥A", 2),
            ("绕过桥B", 3),
        ):
            button = Button(
                self.path_frame,
                text=text,
                height=3,
                font=("Arial", 18),
                command=partial(self.set_switch_path, path_id),
            )
            button.pack(fill="x", pady=6)

        self.control_frame = Frame(self.button_frame)
        self.control_frame.pack(side=RIGHT, fill=Y, padx=(18, 0))

        self.stop_button = Button(
            self.control_frame,
            text="停止",
            width=8,
            height=5,
            font=("Arial", 24, "bold"),
            bg="#d93025",
            fg="white",
            activebackground="#b3261e",
            activeforeground="white",
            command=self.stop_target,
        )
        self.stop_button.pack(fill="x", pady=(0, 14))

        self.start_button = Button(
            self.control_frame,
            text="开始",
            width=8,
            height=5,
            font=("Arial", 24, "bold"),
            bg="#188038",
            fg="white",
            activebackground="#146c2e",
            activeforeground="white",
            command=self.start_target,
        )
        self.start_button.pack(fill="x", pady=(14, 0))

    def run(self):
        self.root.mainloop()

    def close(self):
        self.root.destroy()

    def set_switch_path(self, path_id):
        self.status_label.configure(text=f"正在切换到路径 {path_id}...")
        self.node.set_switch_path(path_id, self.update_status)

    def stop_target(self):
        self.status_label.configure(text="正在停止当前轨迹...")
        self.node.set_target_trigger("stop_target", "停止", self.update_status)

    def start_target(self):
        self.status_label.configure(text="正在开始当前轨迹...")
        self.node.set_target_trigger("start_target", "开始", self.update_status)

    def update_status(self, successful, message):
        def set_text():
            prefix = "成功" if successful else "失败"
            self.status_label.configure(text=f"{prefix}: {message}")

        self.root.after(0, set_text)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGameUiRunNode()

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

    ui = ObstacleGameUiRun(node)
    try:
        ui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
