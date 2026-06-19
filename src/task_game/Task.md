# 任务：实现 Pilot 类

请完成 `Pilot` 类的实现。

接口已经定义在：

```cpp
src/task_game/include/core/pilot.hpp
```

需要实现 `public:` 中已有的所有接口。

如果有必要，可以增加：

- 私有成员变量
- 私有成员函数

但不要修改已有的公共接口定义。

---

# 基本约束

## 1. 不允许修改 TargetPoint

不要修改：

```cpp
Pilot::TargetPoint
```

结构体中的每一个字段都具有明确语义，且后续控制逻辑都会使用。

---

## 2. Pilot 的职责

`Pilot` 是一个自动导航器（Auto Pilot）。

控制循环中会周期性调用：

```cpp
set_state(...)
get_command(...)
```

Pilot 需要根据：

- 当前机器人状态
- 当前轨迹目标

生成机器人控制指令。

---

# 接口要求

## set_target

`set_target(...)` 及其重载函数用于向 Pilot 提交目标轨迹点。

支持：

- 单个轨迹点
- 多个轨迹点

提交后仅保存轨迹。

真正开始执行轨迹需要调用：

```cpp
start()
```

---

## start

调用后：

```cpp
start()
```

Pilot 开始执行当前保存的轨迹。

之后：

```cpp
get_command()
```

需要根据轨迹和当前误差持续生成控制指令。

---

## stop

调用：

```cpp
stop()
```

时：

立即切换到：

- 位控站立模式
- 速度指令全部为 0

即：

```text
vx = 0
vy = 0
omega = 0
```

注意：

停止后不要清除轨迹。

当之后再次调用：

```cpp
start()
```

时，应从之前的执行进度继续执行剩余轨迹。

---

## set_state

用于更新机器人当前状态。

至少需要保存：

- 位置
- 朝向

等导航所需信息。

这些状态将在：

```cpp
get_command()
```

中用于反馈控制。

---

# get_command

需要根据当前状态返回：

- 策略ID
- 速度指令

---

## 没有轨迹任务

如果：

- 没有目标点
- 未启动执行
- 已完成所有轨迹

则返回：

```text
位控站立策略
vx = 0
vy = 0
omega = 0
```

---

## 轨迹执行时的控制律

输出速度应满足：

```text
最终速度 = 轨迹规划速度 + P控制器反馈输出
```

即：

```text
v_cmd = v_ref + kp * error
```

三个自由度分别控制：

- x
- y
- yaw

其中：

```cpp
kp
```

即对应控制器参数。

---

# 导航模式

通过：

```cpp
allow_y_vel
```

决定导航方式。

---

## allow_y_vel == true

允许机器人：

- 边平移
- 边旋转

直接向目标点运动。

即：

- vx
- vy
- omega

可同时非零。

---

## allow_y_vel == false

机器人必须先瞄准目标方向。

### 阶段1：瞄准

持续调整朝向。

只有满足：

```cpp
allow_start_dir_error
```

后才允许开始前进。

即：

```text
|yaw_error| < allow_start_dir_error
```

### 阶段2：前进

开始向目标运动。

前进过程中：

即使出现：

- y方向误差
- yaw误差

也允许直接通过：

- vy
- omega

进行修正。

不要：

- 停车
- 重新进入纯瞄准状态

---

# 轨迹结束后的微调阶段

当轨迹规划时间耗尽后：

不是立即判定完成。

而是进入：

```text
Adjust / Fine Tune
```

状态。

---

## 微调目标

持续调整直到满足：

位置误差：

```cpp
allow_final_pos_allow
```

方向误差：

```cpp
allow_final_dir_error
```

---

## constraint_target_yaw

如果：

```cpp
constraint_target_yaw == true
```

则必须同时满足：

- 位置误差
- 方向误差

才能完成。

如果：

```cpp
constraint_target_yaw == false
```

则只检查位置误差。

---

## 微调最小速度

进入微调阶段后：

```cpp
adjust_min_vel
adjust_min_omega
```

开始生效。

原因：

机器人存在死区。

如果控制器输出小于对应阈值，机器人实际上不会运动。

因此微调阶段需要保证：

```text
|vx| >= adjust_min_vel
|vy| >= adjust_min_vel
|omega| >= adjust_min_omega
```

仅在对应误差非零时进行最小值补偿。

---

# 多轨迹点衔接

对于：

```cpp
set_target(const std::vector<TargetPoint>& target)
```

一次性输入多个轨迹点的情况：

轨迹默认是：

```text
点A -> 点B
点B -> 点C
```

的直线连接。

---

## 问题

在连接点附近会产生：

```text
速度方向突变
```

导致机器人抖动。

---

## 要求

引入：

```cpp
trajectory_connection_radius
```

作为连接半径。

当机器人进入：

```text
距离当前目标点 < trajectory_connection_radius
```

区域后：

不要继续严格跟踪当前线段。

而是提前开始向下一段轨迹过渡。

---

## 过渡方式

使用三次多项式（Cubic Polynomial）

对：

- 位置
- 速度

进行连续过渡。

要求至少满足：

- 位置连续
- 速度连续

避免轨迹连接处出现速度突变。

---

# 推荐状态机

```cpp
enum class PilotState
{
    Idle,
    Running,
    Adjusting,
    Paused,
    Finished
};
```

并维护：

- 当前轨迹索引
- 当前段开始时间
- 执行状态
- 机器人当前位置
- 机器人当前朝向

等必要信息。

---

# 实现风格要求

不要为了“代码看起来高级”而过度拆分函数。

要求：

- 逻辑清晰
- 状态机明确
- 易读易维护

特别注意：

**如果一段代码只在一个地方使用，不要额外抽象成单独函数。**

只有当逻辑会被多处复用时，才考虑抽象为私有成员函数。

避免出现大量：

```cpp
calcA()
calcB()
calcC()
```

这种仅被调用一次的短小包装函数。
