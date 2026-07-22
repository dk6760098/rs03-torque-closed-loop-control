# RS03 Torque Closed-Loop Control

> 安全修正：初始版本把私有协议`run_mode=0`运控帧误标为真正MIT协议，实机会
> 发生高速反向运动。请更新到最新版本；新版本禁止在`motor_protocol=private`时
> 启动`mit_impedance`。

这个ROS2 Humble包提供三种基于力矩的闭环实验。RS03的“私有协议运控模式”与
“MIT标准协议”是两套不同的线上协议，不能混用：

协议格式依据[RS03官方产品资料](https://github.com/RobStride/Product_Information/tree/main/%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/RS03)。

- `velocity_pi`：ROS2节点读取实际速度，用PI计算目标力矩；
- `position_pid`：ROS2节点读取位置和速度，用PID/PD计算目标力矩；
- `mit_impedance`：切换到真正的11位标准帧MIT协议后，把目标位置、速度、
  `Kp`、`Kd`和前馈力矩按16/12位格式打包进MIT复合帧，
  由驱动器高速执行弹簧—阻尼控制。

`velocity_pi`和`position_pid`使用私有协议的29位扩展帧；`mit_impedance`必须使用
MIT协议的11位标准帧。电机协议切换写入后需要断电重启才生效。三种模式最终都
调用驱动器内部电流PI。

## 控制结构

```text
velocity_pi:  速度误差 -> PI + anti-windup -> 私有运控纯力矩帧
position_pid: 位置误差 -> PID + anti-windup -> 私有运控纯力矩帧
mit_impedance:目标位置/Kp/Kd/前馈力矩 ---------------------> MIT复合帧
                                                          -> 内部电流PI
```

`mit_impedance`中的`Kp`表现为刚度，`Kd`表现为阻尼：外力推开电机后会出现
恢复力矩，释放后回到目标附近。它适合柔顺关节和阻抗控制实验。

## 已配置硬件

- ROS2 Humble；
- CH340官方串口转CAN板：`/dev/ttyUSB0`、921600 baud；
- RS03电机ID：12；主机ID：255；
- MIT协议主机反馈ID：0（与私有协议主机ID 255相互独立）；
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

### 1. 从私有协议切换到MIT协议（只执行一次）

当前项目原有电流、力矩、速度和PP测试能够读取`run_mode`，说明电机目前是私有
协议。先确保电机停止，然后执行：

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=false \
  motor_protocol:=private \
  protocol_switch_target:=mit \
  control_mode:=velocity_pi
```

看到`protocol switch command sent`后，彻底断开电机电源，等待数秒，再重新上电。
协议切换会改变后续所有CAN帧格式；未断电前不要继续发送控制命令。

### 2. 只探测MIT标准帧反馈

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=false \
  motor_protocol:=mit \
  mit_host_id:=0 \
  control_mode:=mit_impedance
```

只有看到有效位置、速度、力矩和温度反馈后才能使能。如果没有反馈，立即停止，
不要尝试`auto_enable:=true`。

### 3. MIT低参数测试

终端一：

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=true \
  motor_protocol:=mit \
  mit_host_id:=0 \
  control_mode:=mit_impedance \
  position_max_offset_rad:=0.03 \
  position_tracking_error_rad:=0.08 \
  mit_kp:=1.5 \
  mit_kd:=0.05 \
  mit_feedforward_torque_nm:=0.0 \
  mit_position_slew_rate_rad_s:=0.01 \
  max_torque_nm:=0.20 \
  max_velocity_rad_s:=0.30
```

终端二：

```bash
ros2 topic pub --rate 50 \
  /rs03_torque_closed_loop/position_offset_command_rad \
  std_msgs/msg/Float32 "{data: 0.005}"
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

### 切回私有协议

```bash
ros2 launch rs03_torque_closed_loop_control rs03_torque_closed_loop.launch.py \
  auto_enable:=false \
  motor_protocol:=mit \
  protocol_switch_target:=private \
  control_mode:=mit_impedance
```

看到切换提示后再次彻底断电、等待并重新上电。之后使用
`motor_protocol:=private`运行速度PI或位置PID。

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
