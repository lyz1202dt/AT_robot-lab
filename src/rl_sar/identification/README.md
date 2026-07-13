# ATDog3 关节参数辨识程序使用说明

## 1. 这个目录有什么

```text
identification/
├── atdog3_joint_identification.cpp
├── config/
│   └── atdog3_joint_identification.yaml
└── README.md
```

- `atdog3_joint_identification.cpp`：连接 ATDog3 下位机、生成位置轨迹、执行安全检查并记录 CSV。
- `config/atdog3_joint_identification.yaml`：日常实验需要修改的参数，通常不需要改 C++。
- `README.md`：说明修改位置、运行顺序、实验类型和注意事项。

程序绕过强化学习策略和 FSM，直接向 `LegDriver` 发送位置、Kp、Kd：

```text
YAML实验参数
    ↓
生成目标关节位置 q_cmd
    ↓
LegDriver发送 rad、kp、kd
    ↓
STM32/下位机位置PD控制
    ↓
反馈 q、dq、tau
    ↓
保存CSV
```

本程序不直接发送策略动作，也不发送前馈力矩：

```text
omega = 0
torque = 0
```

## 2. 最常修改哪里

只修改：

```text
config/atdog3_joint_identification.yaml
```

最常用参数：

| 目的 | YAML 参数 |
|---|---|
| 选择实验关节 | `joints.enabled` |
| 设置运行中心角 | `joints.center_rad` |
| 设置实验安全范围 | `joints.lower_limit_rad`、`upper_limit_rad` |
| 设置正弦振幅 | `joints.amplitude_rad` |
| 设置控制增益 | `joints.kp`、`kd` |
| 选择实验类型 | `experiment.type` |
| 设置正弦频率 | `experiment.sine.frequency_hz` |
| 设置阶跃方向和大小 | `experiment.smooth_step.offset_rad` |
| 设置实验时间 | `experiment.duration_seconds` |
| 修改 CSV 路径 | `logging.csv_path` |

通常不需要修改 C++。只有增加新的轨迹类型、修改 CSV 字段或更换硬件协议时才改源码。

## 3. 关节数组顺序

YAML 中所有 12 元数组均使用逻辑顺序：

| 索引 | 关节 |
|---:|---|
| 0 | FR_hip |
| 1 | FR_thigh |
| 2 | FR_calf |
| 3 | FL_hip |
| 4 | FL_thigh |
| 5 | FL_calf |
| 6 | RR_hip |
| 7 | RR_thigh |
| 8 | RR_calf |
| 9 | RL_hip |
| 10 | RL_thigh |
| 11 | RL_calf |

例如只测试右前腿大腿关节：

```yaml
enabled: [false, true, false,
          false, false, false,
          false, false, false,
          false, false, false]
```

### mapping 必须特别核对

`mapping` 表示：

```text
逻辑关节索引 -> 下位机硬件关节索引
```

当前配置为：

```yaml
mapping: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
```

这是假设逻辑顺序和 STM32 返回顺序完全相同。如果实机出现“启用 0 号关节却是另一条腿运动”，应立即急停并修正 `mapping`，不要通过修改 `enabled` 绕过映射错误。

## 4. 三种实验怎么用

### 4.1 hold：第一次必须先运行

作用：

- 验证关节映射和正方向；
- 验证中心角是否正确；
- 验证 Kp/Kd 是否安全；
- 采集静态位置误差、速度噪声和力矩噪声；
- 验证 Ctrl-C 后是否进入安全阻尼。

配置：

```yaml
experiment:
  type: "hold"
  duration_seconds: 5.0
```

程序不会直接跳到中心角，而是：

```text
等待反馈
→ 保持启动实测角
→ 平滑移动到center_rad
→ 保持
→ 安全阻尼退出
```

### 4.2 smooth_step：分析位置响应

作用：

- 上升时间；
- 超调量；
- 稳态误差；
- 正负方向差异；
- Kp/Kd 对响应的影响。

配置示例：

```yaml
experiment:
  type: "smooth_step"
  duration_seconds: 6.0

  smooth_step:
    offset_rad: 0.03
    hold_before_seconds: 2.0
    transition_seconds: 1.0
    hold_after_seconds: 2.0
```

实际目标为：

```text
center_rad
→ 在transition_seconds内平滑变化
→ center_rad + offset_rad
```

该程序故意不发送瞬时硬阶跃，避免实机冲击。需要测试反方向时，将 `offset_rad` 改为负数并单独运行一次。

### 4.3 sine：分析频率响应

作用：

- 测量实际振幅与目标振幅之比；
- 测量相位差；
- 观察频率增加后的跟踪能力；
- 估计执行器延迟、阻尼和带宽。

配置示例：

```yaml
experiment:
  type: "sine"
  duration_seconds: 10.0

  sine:
    frequency_hz: 0.20
    phase_rad: 0.0
```

每个启用关节的目标为：

```text
q_cmd = center_rad + amplitude_rad × sin(2π × frequency_hz × t + phase_rad)
```

第一次建议：

```text
单关节
振幅：0.02~0.03 rad
频率：0.2 Hz
时间：10 s
```

确认安全后按以下顺序单独采集：

```text
0.2 Hz
0.5 Hz
1.0 Hz
1.5 Hz
```

不要第一次就同时启用多个关节，也不要同时提高振幅和频率。

## 5. 中心角和限位怎么填

### center_rad

`center_rad` 是实验围绕的中心位置，不是编码器零点，也不是机械极限。

选择原则：

- 关节在该位置附近有足够正负运动空间；
- 腿不会撞机身、支架、地面或线束；
- 与程序启动时姿态不要相差太大；
- 悬空实验应选择机械负载和重力影响可接受的位置。

### lower_limit_rad / upper_limit_rad

它们是本实验的软限位，应比真实机械极限更保守：

```text
机械可动范围
    └── 实验软限位
           └── 实际轨迹范围
```

正弦必须满足：

```text
center_rad - abs(amplitude_rad) >= lower_limit_rad
center_rad + abs(amplitude_rad) <= upper_limit_rad
```

平滑阶跃必须满足：

```text
lower_limit_rad <= center_rad + offset_rad <= upper_limit_rad
```

程序会在运行前检查上述关系，配置不合法时不会连接实机运动。

## 6. Kp/Kd 怎么调

下位机位置控制通常等价于：

```text
tau = Kp × (q_cmd - q) - Kd × dq
```

### Kp

- 增大：位置跟踪更硬、稳态误差通常减小；
- 过大：冲击、振荡、峰值力矩和发热增加。

### Kd

- 增大：通常增加阻尼、减小振荡；
- 过大：响应变慢，也可能放大速度测量噪声。

建议调节顺序：

1. 只启用一个关节；
2. 使用 `hold`；
3. 从较低 Kp/Kd 开始；
4. 使用 `smooth_step` 观察超调；
5. 每次只修改 Kp 或 Kd 中的一项；
6. 记录每组配置对应的 CSV 文件名。

不要直接把强化学习部署使用的高增益复制进第一次辨识实验。

## 7. 构建和配置校验

构建：

```bash
./build.sh rl_sar
```

如果当前 `install/` 是 isolated 布局而脚本使用 `--merge-install`，需要使用与当前工作区一致的构建方式，或指定临时构建目录。不要直接删除已有 `install/`，其中可能包含其他开发结果。

构建完成后先校验配置：

```bash
ros2 run rl_sar atdog3_joint_identification \
  --validate \
  /绝对路径/atdog3_joint_identification.yaml
```

`--validate` 只读取和检查 YAML，不构造 `LegDriver`，不会发送电机命令。

校验通过后才运行实机：

```bash
ros2 run rl_sar atdog3_joint_identification \
  /绝对路径/atdog3_joint_identification.yaml
```

建议每次复制一份配置，不要覆盖上一组实验参数：

```text
config/
├── joint0_hold.yaml
├── joint0_step_positive.yaml
├── joint0_step_negative.yaml
├── joint0_sine_0p2hz.yaml
└── joint0_sine_0p5hz.yaml
```

## 8. 实机运行前检查

必须满足：

- 机器狗可靠悬空；
- 支架可承受运动和振动；
- 腿的整个轨迹不会撞击物体；
- 线束不会被拉扯；
- 急停可立即触达；
- 现场有人观察；
- 电池电量、温度和电机状态正常；
- 没有同时运行 `rl_real_atdog3` 或其他电机控制程序；
- 已执行 `--validate`；
- 第一次只有一个 `enabled=true`；
- 第一次使用 `hold`；
- 中心角、上下限和 mapping 已逐项核对。

严禁同时运行两个向同一下位机发送命令的程序。

## 9. 程序安全流程

### 启动保护

1. 构造驱动并等待下位机反馈；
2. 未收到反馈时不启用位置控制；
3. 检查初始实测角是否在保护范围；
4. 检查启用关节距离中心角是否过大；
5. 先发送当前实测角；
6. 再平滑移动到中心角。

### 运行保护

每个控制周期检查：

- 目标角是否在软限位内；
- 实测角是否超过软限位和容差；
- 实测速度是否超限；
- 实测力矩是否超限；
- 反馈是否连续丢失；
- 距最后有效反馈是否超时；
- 命令是否发送成功；
- 相邻周期目标角变化是否过大。

### 退出保护

以下情况都会停止实验：

- 实验正常结束；
- Ctrl-C；
- 电机错误回调；
- 反馈丢失；
- 角度、速度或力矩超限；
- 命令发送失败；
- 文件或其他异常。

退出时调用：

```text
enable_control(false)
```

`LegDriver` 随后发送 `kp=0` 和默认 `kd` 的安全阻尼命令。

安全阻尼不是机械急停。程序或电脑完全失去供电、USB 断开时，应依赖硬件急停和下位机自身超时保护。

## 10. CSV 每一列是什么意思

CSV 使用长表格式：每个控制周期写 12 行。

| 字段 | 含义 |
|---|---|
| `steady_time_ns` | PC 单调时钟，单位 ns |
| `elapsed_s` | 从控制循环开始计算的时间 |
| `cycle_index` | 控制周期编号 |
| `phase` | 当前安全状态机阶段 |
| `experiment_type` | hold、smooth_step 或 sine |
| `bottom_time_ms` | 下位机反馈包内的时间戳 |
| `dof` | 逻辑关节索引 |
| `hardware_index` | mapping 后的硬件索引 |
| `leg_index` | 硬件腿编号 |
| `joint_index` | 腿内关节编号 |
| `enabled` | 是否参与实验轨迹 |
| `q_cmd_rad` | 发送目标角 |
| `dq_cmd_rad_s` | 发送目标速度，当前固定为 0 |
| `tau_cmd_nm` | 发送前馈力矩，当前固定为 0 |
| `kp`、`kd` | 下位机位置 PD 增益 |
| `q_meas_rad` | 实测关节角 |
| `dq_meas_rad_s` | 实测关节速度 |
| `tau_meas_nm` | 下位机反馈/估计力矩 |
| `q_error_rad` | `q_cmd_rad - q_meas_rad` |
| `feedback_ok` | 本周期是否成功取得反馈 |
| `set_target_ok` | 本周期命令是否成功发送 |

分析时优先筛选：

```text
phase == RUN_EXPERIMENT
enabled == 1
```

`RAMP_TO_CENTER` 和 `RAMP_BACK` 是安全过渡数据，不应直接混入正弦稳态频响分析。

## 11. C++ 每一部分的作用

### `Config`

保存 YAML 中的全部参数。新增配置项时，需要同步修改：

1. `Config` 字段；
2. `load_config()`；
3. `validate_config()`；
4. YAML 文件；
5. 如需输出，修改 `CsvLogger`。

### `load_config()`

读取 YAML，不负责判断轨迹是否安全。

### `validate_config()`

在连接实机前检查数组长度、映射、时长、增益、中心角、软限位、正弦峰值和阶跃目标。

### `unpack_state()`

把下位机的 `4条腿 × 3关节` 数据按 `mapping` 转换成逻辑 12 DOF 数组。

### `make_targets()`

把逻辑 12 DOF 目标重新映射为下位机结构，并填入位置、Kp、Kd。

### `experiment_command()`

生成 `hold`、`smooth_step` 和 `sine` 的本周期位置目标。需要增加新轨迹类型时主要修改这里，同时扩充配置检查。

### `check_runtime_safety()`

执行每周期的角度、速度和力矩保护。修改阈值应优先改 YAML，不要删除保护代码。

### `SafetyGuard`

保证抛异常或提前退出时也尝试发送安全阻尼命令。

### `CsvLogger`

写入长表 CSV。增加传感器字段时需要修改表头、`write_cycle()` 参数和每行输出。

### `run()`

负责完整实机流程和状态机，是最需要谨慎修改的部分。不要把耗时计算、网络访问或 AI 推理放进 200 Hz 控制循环。

## 12. IDE 报头文件找不到

如果工程已经成功通过 CMake 编译，但 IDE 显示：

```text
'leg_driver/leg_driver.hpp' file not found
```

通常是 clangd 没有读取正确的 `compile_commands.json`，后续连锁报错中的 `std::string`、模板等错误也可能是假错误。

本工程已设置：

```cmake
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

应让 IDE/clangd 使用实际构建目录中的：

```text
build/rl_sar/compile_commands.json
```

如果使用临时构建目录，则对应文件位于：

```text
/tmp/atdog3_ident_build/rl_sar/compile_commands.json
```

可以在 IDE 中重新加载 CMake/clangd，或者把 clangd 的 compilation database 指向该目录。不要为了消除 IDE 假错误，把第三方头文件改成绝对路径。

## 13. 建议的第一轮实验

1. 配置检查：

```text
单关节 enabled=true
type=hold
kp=较低值
kd=较低值
duration=5秒
```

2. 正方向阶跃：

```text
type=smooth_step
offset=+0.03 rad
```

3. 负方向阶跃：

```text
type=smooth_step
offset=-0.03 rad
```

4. 正弦：

```text
amplitude=0.03 rad
frequency=0.2 Hz
```

5. 保持振幅不变，逐次测试：

```text
0.5 Hz
1.0 Hz
1.5 Hz
```

每组使用不同 CSV 文件名，并记录关节、频率、振幅、Kp/Kd、电池电压和异常情况。
