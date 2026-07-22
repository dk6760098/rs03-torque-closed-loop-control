# RS03 Torque Closed-Loop Control

这个ROS2 Humble包让RS03始终工作在`run_mode=0`（Type-1/MIT协议），并提供三种
基于力矩的闭环实验：

- `velocity_pi`：ROS2节点读取实际速度，用PI计算目标力矩；
- `position_pid`：ROS2节点读取位置和速度，用PID/PD计算目标力矩；
- `mit_impedance`：把目标位置、`Kp`、`Kd`和前馈力矩直接放进MIT复合帧，
  由驱动器高速执行弹簧—阻尼控制。

这不是RS03原生速度模式或PP位置模式。三种模式最终都通过MIT/力矩接口调用
驱动器内部电流PI。

## 控制结构

```text
velocity_pi:  速度误差 -> PI + anti-windup -> 力矩限幅/斜坡 -> MIT纯力矩帧
position_pid: 位置误差 -> PID + anti-windup -> 力矩限幅/斜坡 -> MIT纯力矩帧
mit_impedance:目标位置/Kp/Kd/前馈力矩 ---------------------> MIT复合帧
                                                          -> 内部电流PI
```

`mit_impedance`中的`Kp`表现为刚度，`Kd`表现为阻尼：外力推开电机后会出现
恢复力矩，释放后回到目标附近。它适合柔顺关节和阻抗控制实验。

## 已配置硬件

- ROS2 Humble；
- CH340官方串口转CAN板：`/dev/ttyUSB0`、921600 baud；
- RS03电机ID：12；主机ID：255；
- Type-1/Type-2速度量程：±20 rad/s。

如果你的ID或串口不同，先修改`config/rs03_torque_closed_loop.yaml`。

## 安装

```bash
cd ~/ros2_ws/src
git clone https://github.com/dk6760098/rs03-torque-closed-loop-control.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rs03_torque_closed_loop_control
source install/setup.bash
```

每次测试使用两个终端。命令必须持续发布；超过`command_timeout_s`没有新命令，
节点会输出零力矩、停止并锁定，必须重启节点。

## 第一次只检查通信

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=false control_mode:=velocity_pi
```

必须看到有效RS03反馈后，才能进行使能测试。

## 速度PI（上位机速度环，输出力矩）

终端一：

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=true \
  control_mode:=velocity_pi \
  max_velocity_command_rad_s:=0.20 \
  max_torque_nm:=0.50 \
  velocity_kp:=0.40 \
  velocity_ki:=0.30 \
  velocity_integral_limit_nm:=0.20 \
  max_velocity_rad_s:=0.60
```

终端二从很低速度开始：

```bash
ros2 topic pub --rate 50 \
  /rs03_torque_closed_loop/velocity_command_rad_s \
  std_msgs/msg/Float32 "{data: 0.10}"
```

如果静摩擦导致无法起转，只能逐步增加`velocity_integral_limit_nm`、
`velocity_feedforward_nm`或`max_torque_nm`，每次只改一项，并保留超速保护。

## 位置PID（上位机位置环，输出力矩）

终端一：

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=true \
  control_mode:=position_pid \
  position_max_offset_rad:=0.08 \
  position_tracking_error_rad:=0.15 \
  max_torque_nm:=0.40 \
  position_kp:=2.0 \
  position_ki:=0.0 \
  position_kd:=0.20 \
  max_velocity_rad_s:=0.50
```

终端二先测试0.02 rad（约1.15°）：

```bash
ros2 topic pub --rate 50 \
  /rs03_torque_closed_loop/position_offset_command_rad \
  std_msgs/msg/Float32 "{data: 0.02}"
```

位置目标是相对于节点启动位置的偏移，不是任意绝对多圈位置。`position_ki=0`
就是位置PD；只有确认持续负载造成稳态误差后，才小幅增加I项。

## MIT阻抗模式

终端一：

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=true \
  control_mode:=mit_impedance \
  position_max_offset_rad:=0.08 \
  position_tracking_error_rad:=0.15 \
  mit_kp:=2.0 \
  mit_kd:=0.20 \
  mit_feedforward_torque_nm:=0.0 \
  mit_position_slew_rate_rad_s:=0.03 \
  max_torque_nm:=0.40 \
  max_velocity_rad_s:=0.50
```

终端二：

```bash
ros2 topic pub --rate 50 \
  /rs03_torque_closed_loop/position_offset_command_rad \
  std_msgs/msg/Float32 "{data: 0.02}"
```

MIT帧中的理论控制关系为：

```text
torque ≈ Kp * (target_position - position)
       + Kd * (target_velocity - velocity)
       + feedforward_torque
```

当前目标速度固定为0，因此该模式用于位置保持/阻抗实验。节点会通过调整前馈项，
把估算的`P+D+FF`力矩限制在`max_torque_nm`附近；但这不是独立轴端力矩传感器
形成的硬力矩限制，所以超速、温度、位置误差和机械急停仍然必须保留。

## 反馈与诊断

```bash
ros2 topic echo /rs03_torque_closed_loop/velocity_rad_s
ros2 topic echo /rs03_torque_closed_loop/position_rad
ros2 topic echo /rs03_torque_closed_loop/estimated_torque_nm
ros2 topic echo /rs03_torque_closed_loop/commanded_torque_nm
ros2 topic echo /rs03_torque_closed_loop/control_error
ros2 topic hz /rs03_torque_closed_loop/velocity_command_rad_s
ros2 param get /rs03_torque_closed_loop control_mode
```

`control_error`在速度模式下单位为rad/s，在位置和MIT模式下单位为rad。
`commanded_torque_nm`在MIT模式下是根据P、D和前馈计算的估算需求，并非独立
力矩传感器读数。

## 安全设计

- `auto_enable=false`默认不使能；即使设为true，也要等第一条有效命令才使能；
- 命令看门狗；
- 连续反馈丢失停机；
- 力矩输出限幅和变化率限制（MIT使用估算限幅）；
- 连续多帧超速停机；
- 位置跟踪误差停机；
- 过温停机；
- 非有限数命令拒绝；
- 节点退出时发送零力矩并停止。

首次实验必须空载、固定电机本体、清空旋转范围，并准备独立断电手段。不要用手
直接抓旋转轴；柔顺性测试应使用固定测试杆、弹性负载或测力装置。

## 三种模式怎么选

|需求|推荐模式|
|---|---|
|研究速度PI如何根据误差改变力矩|`velocity_pi`|
|研究完整的上位机位置PID/PD|`position_pid`|
|机器人关节弹簧感、阻尼感、柔顺保持|`mit_impedance`|
|严格工业速度或位置运动|优先使用RS03原生速度/位置模式|

普通ROS2节点存在调度和串口抖动，因此本项目用于低速、低力矩外环学习。底层
电流PI仍由RS03驱动器实时执行，不应移到普通ROS2/Python节点中。
