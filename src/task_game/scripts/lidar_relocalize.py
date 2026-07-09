#!/usr/bin/env python3
"""雷达/SLAM 地图到标准地图的 2D 重定位标定工具。

功能：
  - 运行时订阅/查询 TF，默认读取 parent=map, child=lidar_calib_frame 的当前位姿。
  - 对 TF 平移和 yaw 做滑动平均滤波。
  - 每按一次键盘 0，按 MAP_POINTS 的顺序记录一个当前点。
  - 记录满 6 个点后，求出 source 坐标系到标准地图坐标系的 2D 刚体变换。
  - 输出 C++ 可直接粘到 robot.cpp 里用的 x/y/yaw 转换代码。

依赖：
  - ROS2 Python 环境：rclpy, tf2_ros
  - numpy

示例：
  python3 lidar_relocalize.py -dog3
  python3 lidar_relocalize.py -dog2 --dog2-tf 0.12 0 0.092 0 0 0 1
  python3 lidar_relocalize.py -dog3 --parent map --child your_lidar_frame

说明：
  这里的 source 坐标系指你当前雷达/SLAM 给出的 map 坐标系，不是标准比赛地图。
  标定完成后输出 T_standard_source，可把 robot.cpp 中 lookupTransform 得到的 transfer 坐标转换到标准地图。
"""

import argparse
import math
import select
import sys
import termios
import tty
from collections import deque
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


MAP_POINT_NAMES = ["(1,1)", "(4,1)", "(1,2)", "(4,2)", "(2,3)", "(3,3)"]
MAP_POINTS = np.array([
    [1.375, -0.60],
    [1.375,  3.15],
    [2.225, -0.60],
    [2.225,  3.15],
    [5.000, -1.40],
    [5.000,  2.20],
], dtype=float)

# static_tf: base_link -> sensor_frame，即 static_transform_publisher 的 parent=base_link child=sensor_frame。
# 这两个值后面你可以直接改；也可以运行时用 --dog2-tf / --dog3-tf 覆盖。
DOG_STATIC_TF = {
    "dog2": {
        "translation": [0.0, 0.0, 0.0],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
    },
    "dog3": {
        "translation": [0.120, 0.0, 0.092],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
    },
}


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def yaw_to_mat(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


def make_tf_2d(translation, quaternion):
    yaw = quat_to_yaw(*quaternion)
    T = np.eye(3)
    T[:2, :2] = yaw_to_mat(yaw)
    T[:2, 2] = [translation[0], translation[1]]
    return T


def invert_tf_2d(T):
    inv = np.eye(3)
    R = T[:2, :2]
    t = T[:2, 2]
    inv[:2, :2] = R.T
    inv[:2, 2] = -R.T @ t
    return inv


def pose_to_tf_2d(pose):
    T = np.eye(3)
    T[:2, :2] = yaw_to_mat(pose.yaw)
    T[:2, 2] = [pose.x, pose.y]
    return T


def apply_tf(T, pts):
    pts = np.asarray(pts, dtype=float)
    hom = np.hstack([pts, np.ones((len(pts), 1))])
    return (T @ hom.T).T[:, :2]


def kabsch_2d(src, dst):
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)
    P = src - src_c
    Q = dst - dst_c
    H = P.T @ Q
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = dst_c - R @ src_c
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def mean_angle(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


class TerminalKeyReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)


class RelocalizeCalibrator(Node):
    def __init__(self, args):
        super().__init__("lidar_relocalize_calibrator")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples = deque(maxlen=args.filter_window)
        self.recorded_source_base_points = []
        self.last_print_time = self.get_clock().now()

        robot_tf = DOG_STATIC_TF[args.robot]
        self.T_base_sensor = make_tf_2d(robot_tf["translation"], robot_tf["quaternion"])
        self.T_sensor_base = invert_tf_2d(self.T_base_sensor)

        print("\n=== 雷达/SLAM 地图 -> 标准地图 标定 ===")
        print(f"机器人: {args.robot}")
        print(f"查询 TF: parent={args.parent}, child={args.child}")
        print(f"滤波窗口: {args.filter_window}")
        print("静态 TF 使用 base_link -> sensor_frame:")
        print(f"  translation={robot_tf['translation']}")
        print(f"  quaternion ={robot_tf['quaternion']}")
        print("\n按键:")
        print("  0: 记录当前滤波后的点")
        print("  u: 撤销上一个点")
        print("  r: 清空已记录点")
        print("  q: 退出")
        print("\n记录顺序必须严格对应 MAP_POINTS:")
        for i, (name, point) in enumerate(zip(MAP_POINT_NAMES, MAP_POINTS), start=1):
            print(f"  {i}. {name} -> map=({point[0]:.3f}, {point[1]:.3f})")
        print("")

    def lookup_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.args.parent,
            self.args.child,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.args.tf_timeout),
        )
        tr = transform.transform.translation
        rot = transform.transform.rotation
        yaw = quat_to_yaw(rot.x, rot.y, rot.z, rot.w)
        return Pose2D(tr.x, tr.y, yaw)

    def update_sample(self):
        try:
            pose = self.lookup_pose()
        except TransformException as ex:
            now = self.get_clock().now()
            if (now - self.last_print_time).nanoseconds > 1_000_000_000:
                print(f"等待 TF {self.args.parent}->{self.args.child}: {ex}")
                self.last_print_time = now
            return None
        self.samples.append(pose)
        return self.filtered_pose()

    def filtered_pose(self):
        if not self.samples:
            return None
        return Pose2D(
            x=sum(p.x for p in self.samples) / len(self.samples),
            y=sum(p.y for p in self.samples) / len(self.samples),
            yaw=mean_angle([p.yaw for p in self.samples]),
        )

    def record_current_point(self):
        pose = self.filtered_pose()
        if pose is None:
            print("还没有可用 TF，无法记录。")
            return False
        if len(self.recorded_source_base_points) >= len(MAP_POINTS):
            print("6 个点已经记录完成，如需重来按 r。")
            return False

        # lookup 得到的是 source_map -> sensor_frame。
        # 转成 source_map -> base_link，得到 base_link 在当前雷达/SLAM 地图中的坐标。
        T_source_sensor = pose_to_tf_2d(pose)
        T_source_base = T_source_sensor @ self.T_sensor_base
        source_base_xy = T_source_base[:2, 2].copy()
        source_base_yaw = math.atan2(T_source_base[1, 0], T_source_base[0, 0])

        self.recorded_source_base_points.append(source_base_xy)
        idx = len(self.recorded_source_base_points)
        map_pt = MAP_POINTS[idx - 1]
        print(
            f"记录 {idx}/{len(MAP_POINTS)} {MAP_POINT_NAMES[idx - 1]}: "
            f"source_base=({source_base_xy[0]:.4f}, {source_base_xy[1]:.4f}, yaw={source_base_yaw:.4f}) "
            f"-> standard_map=({map_pt[0]:.4f}, {map_pt[1]:.4f})"
        )

        if len(self.recorded_source_base_points) == len(MAP_POINTS):
            self.solve_and_print()
        return True

    def undo(self):
        if not self.recorded_source_base_points:
            print("没有可撤销的点。")
            return
        self.recorded_source_base_points.pop()
        print(f"已撤销，当前 {len(self.recorded_source_base_points)}/{len(MAP_POINTS)}。")

    def reset(self):
        self.recorded_source_base_points.clear()
        print("已清空记录点。")

    def solve_and_print(self):
        source_points = np.array(self.recorded_source_base_points, dtype=float)
        T_standard_source = kabsch_2d(source_points, MAP_POINTS)
        pred = apply_tf(T_standard_source, source_points)
        err = np.linalg.norm(pred - MAP_POINTS, axis=1)
        yaw = math.atan2(T_standard_source[1, 0], T_standard_source[0, 0])
        x = T_standard_source[0, 2]
        y = T_standard_source[1, 2]

        np.set_printoptions(precision=8, suppress=True)
        print("\n=== 标定结果 ===")
        print("T_standard_source (当前雷达/SLAM地图坐标 -> 标准地图坐标):")
        print(T_standard_source)
        print(f"\n二维参数: x={x:.8f}, y={y:.8f}, yaw={yaw:.8f} rad, yaw_deg={math.degrees(yaw):.4f}")
        print("\n逐点残差:")
        for i, e in enumerate(err):
            print(
                f"  {i + 1}. {MAP_POINT_NAMES[i]} "
                f"source=({source_points[i,0]:.4f},{source_points[i,1]:.4f}) "
                f"pred=({pred[i,0]:.4f},{pred[i,1]:.4f}) "
                f"map=({MAP_POINTS[i,0]:.4f},{MAP_POINTS[i,1]:.4f}) "
                f"err={e:.4f} m"
            )
        print(f"RMSE={math.sqrt(float((err ** 2).mean())):.4f} m, max_err={float(err.max()):.4f} m")

        print("\n=== 可粘到 robot.cpp 的转换代码 ===")
        print("// 把当前 lookupTransform 得到的雷达/SLAM map 坐标转换到标准地图坐标")
        print(f"const double relocalize_cos = {math.cos(yaw):.12f};")
        print(f"const double relocalize_sin = {math.sin(yaw):.12f};")
        print(f"const double relocalize_tx = {x:.12f};")
        print(f"const double relocalize_ty = {y:.12f};")
        print("const double source_x = transfer.transform.translation.x;")
        print("const double source_y = transfer.transform.translation.y;")
        print("transfer.transform.translation.x = relocalize_cos * source_x - relocalize_sin * source_y + relocalize_tx;")
        print("transfer.transform.translation.y = relocalize_sin * source_x + relocalize_cos * source_y + relocalize_ty;")
        print("// z 通常不参与 2D 地图重定位；如需高度偏移，可单独加常量。")
        print("\n标定完成后可以按 q 退出；如需重采按 r。\n")


def parse_args():
    parser = argparse.ArgumentParser(description="采集 TF 点位并求雷达/SLAM地图到标准地图的 2D 变换")
    robot_group = parser.add_mutually_exclusive_group(required=True)
    robot_group.add_argument("-dog2", dest="robot", action="store_const", const="dog2", help="使用 dog2 静态 TF")
    robot_group.add_argument("-dog3", dest="robot", action="store_const", const="dog3", help="使用 dog3 静态 TF")
    parser.add_argument("--parent", default="map", help="被查询 TF 的父坐标系，默认 map")
    parser.add_argument("--child", default="mid360_imu", help="被查询 TF 的子坐标系，后面你可以改，默认 mid360_imu")
    parser.add_argument("--filter-window", type=int, default=20, help="滑动平均滤波窗口，默认 20")
    parser.add_argument("--tf-timeout", type=float, default=0.05, help="TF 查询超时秒数，默认 0.05")
    parser.add_argument("--dog2-tf", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"), help="运行时覆盖 dog2 的 base_link->sensor 静态 TF")
    parser.add_argument("--dog3-tf", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"), help="运行时覆盖 dog3 的 base_link->sensor 静态 TF")
    args = parser.parse_args()

    if args.dog2_tf:
        DOG_STATIC_TF["dog2"] = {"translation": args.dog2_tf[:3], "quaternion": args.dog2_tf[3:]}
    if args.dog3_tf:
        DOG_STATIC_TF["dog3"] = {"translation": args.dog3_tf[:3], "quaternion": args.dog3_tf[3:]}
    if args.filter_window < 1:
        parser.error("--filter-window 必须 >= 1")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = RelocalizeCalibrator(args)
    try:
        with TerminalKeyReader() as key_reader:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                pose = node.update_sample()
                key = key_reader.read_key()
                if key == "0":
                    node.record_current_point()
                elif key == "u":
                    node.undo()
                elif key == "r":
                    node.reset()
                elif key == "q":
                    break

                now = node.get_clock().now()
                if pose is not None and (now - node.last_print_time).nanoseconds > 1_000_000_000:
                    print(
                        f"当前滤波 TF {args.parent}->{args.child}: "
                        f"x={pose.x:.4f}, y={pose.y:.4f}, yaw={pose.yaw:.4f}; "
                        f"已记录 {len(node.recorded_source_base_points)}/{len(MAP_POINTS)}，按 0 记录"
                    )
                    node.last_print_time = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
