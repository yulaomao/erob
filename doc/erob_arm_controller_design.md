# eRob 三轴机械臂控制类设计方案

## 1. 文档目标

本文档面向当前仓库的后续开发，目标是在现有 SOEM 能力和 demo 基础上，设计一套可复用、可扩展、适用于 3 轴机械臂的 eRob 控制类方案。

本文档解决以下问题：

- 如何从当前单 demo 程序演进为可复用控制类。
- 如何实现首次自动扫描网卡并识别可用电机。
- 如何管理 3 个逻辑轴与 EtherCAT 从站之间的绑定关系。
- 如何实现位置模式与随动模式。
- 如何处理使能、故障恢复、状态查询、线程调度与实时循环。
- 如何安排分阶段开发与验证。

本文档不直接替代驱动器说明书。具体对象字典、控制字位定义、错误码含义仍需以 eRob 驱动器手册为准。

---

## 1.1 审查修订记录

| 日期 | 修订内容 |
|------|----------|
| 2026-04-14 | 初版审查优化：修复 8 项设计问题，增加性能优化设计，增加全模式位置限位 |

本次审查修复的主要问题：

1. **AxisMode 枚举混合问题**：原设计将运动模式（PP、CSV）和设备状态（Fault、Disabled）混在同一枚举中，已拆分为 MotionMode + CiA402State。
2. **PDO 切换策略错误**：原建议方案 A（按模式重映射 PDO）需回退到 PRE-OP，中断控制。已改为方案 B（统一混合 RxPdoUnified）。
3. **TxPdo 缺少 mode_of_operation_display**：无法确认驱动器实际生效模式，已补充。
4. **线程同步用 mutex 不安全**：1kHz 实时线程中使用互斥锁可能导致优先级反转。已改为原子双缓冲无锁方案。
5. **CSV 随动模式缺位置限位**：CSV 发速度命令，驱动不会自动限位，实际位置可能超限。已增加运行时位置限位器（§16.7）。
6. **60Hz 到 1kHz 速度阶梯问题**：原设计未说明两个频率之间的插值策略。已增加线性插值方案（§16.8）。
7. **随动模式缺看门狗**：上层停止更新目标但未停止模式时，电机会以最后速度持续运行。已增加进门狗机制（§16.9）。
8. **所有轴限位统一为 -130° 到 130°**：原设计各轴限位不一致且默认值过大。已统一修改。

增加的性能优化设计：

- 无锁双缓冲命令与状态传递
- cache line 对齐避免 false sharing
- 内存预分配策略
- 速度插值减少力矩波动
- iomap 缓冲区扩展至 8KB

---

## 2. 现状分析

### 2.1 当前仓库已有能力

当前仓库已经具备以下基础：

- SOEM 主站能力。
- EtherCAT 从站扫描、状态切换、PDO 映射、周期收发。
- eRob 多种运行模式 demo，包括：
  - PP
  - CSP
  - CSV
  - CST
  - PT
- 基于 CiA402 状态机的控制字切换流程。
- DC 同步、工作计数检查、从站恢复逻辑。

### 2.2 当前 demo 的局限

现有 demo 主要用于单功能验证，不适合作为三轴控制类的最终形态，主要问题如下：

- 网卡名硬编码，缺少自动扫描。
- 大量全局变量，不适合封装成对象。
- 多个从站共享同一组发送和接收结构，无法形成真正的多轴独立控制。
- 轨迹规划器只有一套全局实例，无法支持每个轴独立模式。
- 网络扫描、PDO 配置、使能控制、运动控制、服务接口混杂在同一个文件中。
- demo 更像一次性运行程序，而非长期运行的控制组件。

因此，必须从结构上重构，而不是继续在某一个 demo 文件上叠加逻辑。

---

## 3. 总体设计目标

### 3.1 功能目标

控制类需要满足以下目标：

1. 首次初始化自动扫描本机网卡，识别可用 EtherCAT 电机。
2. 将识别结果记录下来，并支持后续重新扫描更新。
3. 正常情况下控制 3 个电机，对每个电机可独立操作。
4. 提供位置模式：指定轴 id、目标角度、目标速度，控制电机运动到目标位置。
5. 提供随动模式：指定轴 id、目标角度，系统以 60 Hz 进行目标更新，并根据当前位置和速度计算下一段速度与加速度，使其尽可能快且稳定地跟随目标。
6. 提供基础控制：使能、失能、清故障、急停、状态读取。
7. 提供掉线检测、WKC 检查、从站恢复。

### 3.2 工程目标

- 不依赖单一 demo 程序结构。
- 业务逻辑与 EtherCAT 总线处理解耦。
- 每个轴的状态、命令、限制和模式独立管理。
- 后续可扩展到更多轴数。
- 后续可扩展到关节同步控制和上层接口，例如 TCP、ROS、GUI。

### 3.3 非目标

第一阶段不做以下内容：

- 复杂逆运动学。
- 笛卡尔空间轨迹规划。
- 力控、阻抗控制。
- 多主站冗余。
- 在线参数整定界面。

---

## 4. 设计原则

### 4.1 分层设计

将系统拆为 3 层：

1. EtherCAT 通信层
2. 电机轴控制层
3. 三轴机械臂控制层

### 4.2 软硬件职责分离

- SOEM 层负责网络、从站、PDO、SDO、周期收发。
- 轴控制层负责单轴状态机、单位换算、模式控制、限位检查。
- 机械臂控制层负责三轴编组、轴绑定、统一接口、系统级安全策略。

### 4.3 实时和非实时分离

- 周期收发线程必须只做轻量工作。
- 扫描、持久化、日志、配置加载应在非实时线程完成。
- 随动轨迹计算频率可为 60 Hz，但 EtherCAT 总线周期应保持更高频率，例如 1 kHz。

### 4.4 安全优先

- 所有运动指令必须先通过软限位检查。
- 任何模式切换必须在状态合法时进行。
- 任意单轴严重故障可配置为全轴急停。

---

## 5. 推荐总体架构

建议引入以下核心类：

### 5.1 EthercatMasterSession

职责：

- 扫描本机网卡。
- 连接指定网卡。
- 扫描 EtherCAT 从站。
- 初始化 SOEM 上下文。
- 配置 PDO。
- 切换从站状态。
- 启动周期线程。
- 提供 SDO/PDO 读写能力。
- 处理通信错误与重连。

### 5.2 ErobAxis

职责：

- 表示单个 eRob 电机轴。
- 持有该轴的配置、运行状态和当前命令。
- 处理该轴的 CiA402 基础动作。
- 实现位置模式和随动模式的单轴逻辑。

### 5.3 ErobArmController

职责：

- 管理最多 3 个逻辑轴。
- 完成电机发现与轴绑定。
- 对外提供统一控制接口。
- 管理全局运行状态。
- 提供系统级急停、全使能、全失能、状态汇总。

### 5.4 ConfigManager

职责：

- 读写配置文件。
- 保存扫描结果。
- 保存轴绑定关系。
- 保存计数换算参数、软限位、home offset、跟随控制参数。

---

## 6. 推荐目录结构

建议新增一个独立模块，而不是继续扩展 demo 文件：

```text
demo/
  erob_arm_demo.cpp

include/erob/
  ethercat_master_session.h
  erob_axis.h
  erob_arm_controller.h
  erob_types.h
  erob_config.h

src/erob/
  ethercat_master_session.cpp
  erob_axis.cpp
  erob_arm_controller.cpp
  erob_config.cpp

config/
  erob_arm.yaml
  discovered_motors.json
```

其中：

- demo 只负责演示调用。
- include 和 src 负责类实现。
- config 负责本地配置与设备记录。

---

## 7. 数据模型设计

### 7.1 网卡信息 AdapterInfo

```cpp
struct AdapterInfo {
    std::string name;
    std::string description;
    bool scan_success = false;
    int discovered_slave_count = 0;
};
```

### 7.2 电机身份信息 MotorIdentity

```cpp
struct MotorIdentity {
    std::string adapter_name;
    uint16_t slave_index = 0;
    uint16_t alias = 0;
    uint32_t eep_man = 0;
    uint32_t eep_id = 0;
    uint32_t eep_rev = 0;
    std::string name;
    std::string serial_number;
    bool is_erob_motor = false;
};
```

说明：

- 如果驱动支持读取序列号，应优先将序列号作为设备唯一标识。
- 如果序列号无法读取，则至少保存 eep_man、eep_id、eep_rev 与初次拓扑位置。

### 7.3 轴配置 AxisConfig

```cpp
struct AxisConfig {
    int logical_axis_id = -1;
    std::string joint_name;
    double counts_per_degree = 1.0;
    double home_offset_deg = 0.0;
    int32_t home_offset_count = 0;
    double min_angle_deg = -130.0;
    double max_angle_deg = 130.0;
    double max_velocity_deg_s = 180.0;
    double max_accel_deg_s2 = 360.0;
    double max_decel_deg_s2 = 360.0;
    MotorIdentity bound_motor;
};
```

说明：

- 每个轴必须独立配置 counts_per_degree。
- 不建议所有轴共用同一换算系数。
- 实际三轴通常限位不同，最终应为每个关节独立设置最小和最大角度。

### 7.4 轴状态 AxisState

```cpp
// 运动模式枚举（对应 CiA402 mode_of_operation）
enum class MotionMode {
    kNone,                    // 未设置模式
    kProfilePosition,         // PP, mode = 1
    kCyclicSyncVelocity,      // CSV, mode = 9
    kCyclicSyncPosition,      // CSP, mode = 8（第二阶段预留）
    kCyclicSyncTorque          // CST, mode = 10（第二阶段预留）
};

// CiA402 驱动状态枚举（由 statusword 解析得到）
enum class CiA402State {
    kNotReadyToSwitchOn,
    kSwitchOnDisabled,
    kReadyToSwitchOn,
    kSwitchedOn,
    kOperationEnabled,
    kQuickStopActive,
    kFaultReactionActive,
    kFault
};

struct AxisState {
    bool online = false;
    bool enabled = false;
    bool fault = false;
    uint16_t statusword = 0;
    uint16_t controlword = 0;
    MotionMode motion_mode = MotionMode::kNone;
    CiA402State cia402_state = CiA402State::kSwitchOnDisabled;

    int32_t actual_position_count = 0;
    int32_t actual_velocity_count_s = 0;
    int16_t actual_torque = 0;

    double actual_angle_deg = 0.0;
    double actual_velocity_deg_s = 0.0;

    bool near_positive_limit = false;
    bool near_negative_limit = false;

    int al_status_code = 0;
    int last_error_code = 0;
};
```

说明：

- 之前的设计将运动模式（PP、CSV）和设备状态（Fault、Disabled）混合在同一个枚举中，不利于逻辑判定。
- 拆分为 MotionMode 和 CiA402State 后，运动模式切换与状态机判断解耦，代码更清晰。
- near_positive_limit / near_negative_limit 用于运行时位置监控，便于上层快速判断是否接近限位。

### 7.5 轴命令 AxisCommand

```cpp
struct AxisCommand {
    MotionMode requested_mode = MotionMode::kNone;
    double target_angle_deg = 0.0;
    double target_velocity_deg_s = 0.0;
    double target_accel_deg_s2 = 0.0;
    bool command_updated = false;
};
```

### 7.6 随动模式参数 FollowParams

```cpp
struct FollowParams {
    double control_rate_hz = 60.0;
    double kp = 8.0;
    double kd = 0.15;
    double deadband_deg = 0.05;
    double max_velocity_deg_s = 300.0;
    double max_accel_deg_s2 = 800.0;
    double max_decel_deg_s2 = 800.0;
    double target_filter_alpha = 0.4;
    double watchdog_timeout_ms = 50.0;
    double position_limit_margin_deg = 5.0;
};
```

说明：

- watchdog_timeout_ms：若 60Hz 规划线程在此时间内未产生新的目标更新，则自动开始减速停止。防止通信或线程异常导致速度命令无限持续。
- position_limit_margin_deg：进入限位减速区的提前量。当实际位置距离软限位不足该值时，按比例削减同向速度命令。

---

## 8. 模式选择建议

### 8.1 位置模式建议优先采用 PP

位置模式建议第一阶段使用 Profile Position 模式，而不是直接采用 CSP。

原因：

- 需求语义是给定位置和速度，到达目标点。
- PP 更符合驱动内部完成加减速曲线的思路。
- 当前仓库已有 PP demo，可直接借鉴对象字典配置和触发流程。
- 第一阶段实现复杂度更低，联调成本更小。

位置模式使用：

- 目标位置对象：0x607A
- 模式对象：0x6060 = 1
- 轮廓速度：0x6081
- 加速度：0x6083
- 减速度：0x6084

### 8.2 随动模式建议第一阶段采用 CSV

随动模式建议第一阶段使用 Cyclic Synchronous Velocity，而不是直接采用 CSP。

原因：

- 需求明确按 60 Hz 更新目标，并计算下一时刻速度和加速度。
- 这种控制逻辑更适合输出速度指令。
- 当前仓库已有 CSV demo，可复用 PDO 映射。
- 与主站侧离散控制器耦合更自然。

随动模式使用：

- 目标速度对象：0x60FF
- 模式对象：0x6060 = 9

### 8.3 后续演进方向

第二阶段可视需要增加 CSP 版本跟随控制：

- 适合更高精度位置插补。
- 更适合未来做三轴同步轨迹。
- 但主站端需要更严格的周期稳定性与轨迹连续性。

因此建议路线为：

1. 第一阶段：PP + CSV
2. 第二阶段：CSP 替代或并行支持

---

## 9. PDO 设计建议

### 9.1 单轴 PDO 抽象

建议将每个轴的 PDO 数据独立保存，而不是所有轴共享一个全局结构。

#### 位置模式发送 PDO

```cpp
struct RxPdoPosition {
    uint16_t controlword;
    int32_t target_position;
    uint8_t mode_of_operation;
    uint8_t padding;
} __attribute__((packed));
```

#### 随动模式发送 PDO

```cpp
struct RxPdoVelocity {
    uint16_t controlword;
    int32_t target_velocity;
    uint8_t mode_of_operation;
    uint8_t padding;
} __attribute__((packed));
```

#### 接收 PDO

```cpp
struct TxPdoCommon {
    uint16_t statusword;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
    int8_t mode_of_operation_display;
    uint8_t padding;
} __attribute__((packed));
```

### 9.2 推荐策略

统一使用一种接收 PDO 结构，发送 PDO 结构根据当前模式切换。

实际实现可采用以下方式之一：

- 方案 A：按模式分别重新映射 PDO（需要回到 PRE-OP 状态重新配置，运行时切换代价极高）
- 方案 B：使用统一的混合 PDO，同时包含 target_position 和 target_velocity

建议采用方案 B。原因如下：

- 方案 A 在模式切换时需要将从站回退到 PRE-OP 重新映射 PDO，会中断实时控制，切换期间总线不可用，代价过高。
- 方案 B 只是多占用几个字节 IO 空间，对 1kHz 周期性能影响可忽略。
- 方案 B 允许在 OPERATIONAL 状态下仅通过修改 mode_of_operation 即可切换模式，无需中断总线。
- 未用到的目标字段写零即可，驱动根据当前生效模式取对应目标。

统一发送 PDO 结构建议如下：

```cpp
struct RxPdoUnified {
    uint16_t controlword;
    int32_t target_position;
    int32_t target_velocity;
    int8_t mode_of_operation;
    uint8_t padding;
} __attribute__((packed));
```

运行时根据当前模式，仅填充对应的目标字段，另一个写零。

---

## 10. 网卡扫描与设备发现方案

### 10.1 网卡扫描

SOEM 已支持本机网卡枚举，可通过 ec_find_adapters 或对应 ecx 接口获取适配器列表。

扫描流程建议如下：

1. 获取所有本机网卡。
2. 过滤明显不可能用于 EtherCAT 的接口，例如 lo。
3. 对每个候选网卡尝试执行主站初始化。
4. 调用从站扫描。
5. 如果发现从站，则记录该网卡为可用网卡。
6. 读取每个从站的基本身份信息。

### 10.2 电机识别

识别某个从站是否为 eRob 电机，建议按以下优先级判断：

1. 序列号或专有对象字典标识
2. eep_man + eep_id + eep_rev
3. 从站名称

建议不要只依赖从站名称字符串，因为固件版本、语言和设备别名都可能导致名称变化。

### 10.3 发现结果持久化

首次扫描后的发现结果写入本地文件：

```json
{
  "last_adapter": "enp6s0",
  "motors": [
    {
      "serial_number": "EROB-A001",
      "adapter_name": "enp6s0",
      "slave_index": 1,
      "eep_man": 12345,
      "eep_id": 67890,
      "eep_rev": 1,
      "name": "eRob Servo"
    }
  ]
}
```

### 10.4 重扫更新

重扫时分为两类情况：

- 仅刷新当前网卡下的从站信息
- 重新扫描全部网卡

建议提供两个接口：

- rescanCurrentAdapter
- rescanAllAdapters

---

## 11. 轴绑定设计

### 11.1 为什么需要逻辑轴绑定

EtherCAT slave index 反映的是总线物理顺序，不等价于机械臂关节语义。必须引入逻辑轴绑定，将三轴分别定义为：

- 轴 0：base
- 轴 1：shoulder
- 轴 2：elbow

### 11.2 绑定方式

建议绑定信息写入配置文件：

```yaml
axes:
  - logical_axis_id: 0
    joint_name: base
    motor_serial: EROB-A001
    counts_per_degree: 1456.3
    home_offset_count: 0
    min_angle_deg: -130.0
    max_angle_deg: 130.0

  - logical_axis_id: 1
    joint_name: shoulder
    motor_serial: EROB-A002
    counts_per_degree: 1820.0
    home_offset_count: 100
    min_angle_deg: -130.0
    max_angle_deg: 130.0

  - logical_axis_id: 2
    joint_name: elbow
    motor_serial: EROB-A003
    counts_per_degree: 1820.0
    home_offset_count: -50
    min_angle_deg: -130.0
    max_angle_deg: 130.0
```

### 11.3 启动时绑定流程

1. 读取配置文件。
2. 完成网卡和从站扫描。
3. 依据 motor_serial 或身份信息进行匹配。
4. 构造 3 个 ErobAxis 对象。
5. 若缺少某轴，则系统进入 degraded 状态。

---

## 12. 线程与调度模型

### 12.1 推荐线程划分

建议至少拆分为 4 个线程：

1. 管理线程
2. EtherCAT 周期线程
3. 随动规划线程
4. 监控恢复线程

### 12.2 管理线程

职责：

- 加载配置
- 扫描网卡
- 发现设备
- 绑定逻辑轴
- 启停系统
- 接收外部命令

该线程非实时。

### 12.3 EtherCAT 周期线程

职责：

- 按固定周期执行 send/receive processdata
- 更新每个轴的接收 PDO
- 根据当前命令写入每个轴的发送 PDO
- 完成 CiA402 基础状态维护
- 检查 WKC 和 DC 同步状态

建议周期：

- 1 kHz，优先推荐
- 或 2 kHz，视系统稳定性而定

### 12.4 随动规划线程

职责：

- 每秒 60 次读取最新轴状态
- 基于目标角度、当前位置和当前速度计算下一帧命令速度
- 将计算结果写入共享命令缓冲区

建议周期：

- 60 Hz

### 12.5 监控恢复线程

职责：

- 检查从站状态
- 发现 SAFE_OP、ERROR、lost 等异常
- 执行 reconfig 或 recover
- 上报系统健康状态

建议周期：

- 10 Hz 到 50 Hz

### 12.6 锁与无锁设计建议

1kHz 实时线程中使用互斥锁存在优先级反转和延迟不可控的风险。建议采用无锁方案：

- 命令传递：使用原子双缓冲（double buffer + atomic flag）。60Hz 规划线程写入 back buffer，完成后原子翻转。1kHz 线程始终读 front buffer，无竞争。
- 状态反馈：1kHz 线程写入 back buffer 后原子翻转。60Hz 规划线程和管理线程读 front buffer。
- 若必须使用锁，限定为 std::atomic_flag 自旋锁，锁持有时间不超过几百纳秒，且仅保护单次内存拷贝。
- 绝对禁止在 1kHz 线程中使用 std::mutex、条件变量或任何可能触发内核调度的同步原语。

补充要求：

- 周期线程内禁止文件 IO、内存分配（malloc/new）、字符串拼接和复杂日志。
- 所有缓冲区、数据结构必须在初始化阶段预分配。
- 每轴的 PDO 数据和状态结构建议 cache line 对齐（alignas(64)），避免多轴写入时产生 false sharing。

---

## 13. CiA402 状态机设计

### 13.1 基础动作封装

建议不要在上层业务中直接写控制字，而是封装以下动作：

- resetFault
- shutdown
- switchOn
- enableOperation
- disableOperation
- disableVoltage
- quickStop
- halt

### 13.2 建议接口

```cpp
bool resetFault(int axis_id);
bool enableAxis(int axis_id);
bool disableAxis(int axis_id);
bool quickStopAxis(int axis_id);
bool quickStopAll();
```

### 13.3 使能流程

典型使能流程：

1. 若 fault，则先 reset fault
2. 控制字切到 shutdown
3. 控制字切到 switch on
4. 控制字切到 enable operation
5. 确认 statusword 进入 operation enabled

### 13.4 模式切换规则

建议约束如下：

- 未使能前不得进入运动模式
- 模式切换时应先将目标量清零或置当前值
- 从位置模式切换到随动模式时，应先将速度命令初始化为 0
- 从随动模式切换到位置模式时，应先固定当前位置，再发新位置目标

---

## 14. 单位换算设计

### 14.1 角度到编码器计数

统一定义：

```text
count = home_offset_count + angle_deg * counts_per_degree
```

### 14.2 速度换算

```text
velocity_count_s = velocity_deg_s * counts_per_degree
```

### 14.3 位置反馈换算

```text
angle_deg = (actual_position_count - home_offset_count) / counts_per_degree
```

### 14.4 注意事项

- counts_per_degree 必须按轴独立配置。
- 若存在减速器反向安装，应支持负系数或方向位配置。
- home offset 必须支持标定保存。

---

## 15. 位置模式设计

### 15.1 用户接口

```cpp
bool moveTo(int axis_id, double target_angle_deg, double velocity_deg_s);
```

### 15.2 输入约束

- axis_id 范围合法
- 电机在线
- 轴已绑定
- 轴已使能
- 目标角度在软限位内
- 速度不超过该轴最大允许值

### 15.3 执行流程

1. 检查状态和参数。
2. 将目标角度换算为 target_position_count。
3. 将速度换算为 profile velocity。
4. 写入 0x6081、0x6083、0x6084。
5. 设置模式 1。
6. 通过控制字发起新的 set-point。
7. 监测到位或异常退出。

### 15.4 到位判定

建议同时使用以下条件：

- 位置误差小于阈值
- 速度绝对值小于阈值
- 驱动状态字显示目标到达或运动结束位

### 15.5 位置模式状态机

建议状态：

- Idle
- WaitingEnable
- SendingSetpoint
- Moving
- TargetReached
- Timeout
- Fault

---

## 16. 随动模式设计

### 16.1 用户接口

```cpp
bool startFollowMode(int axis_id);
bool updateFollowTarget(int axis_id, double target_angle_deg);
bool stopFollowMode(int axis_id);
```

### 16.2 运行机制

随动模式由两个速率共同组成：

- 总线周期线程：1 kHz，下发速度指令
- 跟随规划线程：60 Hz，计算下一帧目标速度

### 16.3 控制目标

输入目标角度不断变化时，使电机尽可能快跟踪，但要满足：

- 不超速度限制
- 不超加速度限制
- 不剧烈震荡
- 目标停止变化时能平稳停下

### 16.4 推荐离散控制器

设：

- 目标角度为 q_target
- 当前角度为 q
- 当前速度为 v
- 误差为 e = q_target - q
- 采样周期为 dt = 1/60

推荐控制流程：

1. 对输入目标做一阶滤波，降低突变：

```text
q_target_filtered = alpha * q_target + (1 - alpha) * q_target_prev
```

2. 计算误差：

```text
e = q_target_filtered - q
```

3. 若误差小于死区，则目标速度设为 0。

4. 否则计算目标速度：

```text
v_des = kp * e - kd * v
```

5. 速度限幅：

```text
v_des = clamp(v_des, -v_max, v_max)
```

6. 加速度限幅：

```text
a = (v_des - v_prev) / dt
a = clamp(a, -a_max, a_max)
v_cmd = v_prev + a * dt
```

7. 再换算为 count/s 并写入 0x60FF。

### 16.5 跟随模式状态机

建议状态：

- Idle
- WaitingEnable
- EnteringCsvMode
- Following
- Stopping
- Fault

### 16.6 为什么不建议第一版直接用 CSP

如果直接用 CSP，需要主站在较高频率下持续输出位置轨迹点，对周期抖动和轨迹连续性要求更高。对于当前项目而言，CSV 能更快形成稳定版本，并且满足你提出的 60 Hz 跟随目标更新要求。

### 16.7 随动模式运行时位置限位

CSV 模式下驱动器接收速度指令，不会自动执行位置限位。如果仅在命令入口检查目标角度，实际位置仍可能因惯性、累积误差或外力超过限位。必须在 1kHz 周期线程中实施运行时位置限位器。

限位器逻辑：

```text
margin = position_limit_margin_deg  // 默认 5.0°

// 接近正方向限位
if actual_angle > max_angle - margin:
    remaining = max_angle - actual_angle
    scale = clamp(remaining / margin, 0, 1)
    if v_cmd > 0:
        v_cmd = v_cmd * scale

// 接近负方向限位
if actual_angle < min_angle + margin:
    remaining = actual_angle - min_angle
    scale = clamp(remaining / margin, 0, 1)
    if v_cmd < 0:
        v_cmd = v_cmd * scale

// 已超过限位：强制速度归零
if actual_angle >= max_angle:
    v_cmd = min(v_cmd, 0)  // 只允许反向
if actual_angle <= min_angle:
    v_cmd = max(v_cmd, 0)  // 只允许反向
```

该限位器在周期线程中位于 PD 控制器输出之后、PDO 写入之前，是最后一道安全闸门。

### 16.8 60Hz 到 1kHz 速度插值

60Hz 规划器每次产生一个速度命令，但 1kHz 总线周期之间有约 16-17 个周期的间隔。若简单保持上一次速度命令不变，会产生阶梯形速度曲线，导致电机力矩波动和机械报动。

建议在 1kHz 线程中做线性插值：

```text
// 每次 60Hz 更新时记录
v_start = v_prev_cmd
v_end = v_new_from_planner
interp_steps = cycles_between_planner_updates  // 约 16-17
interp_count = 0

// 每个 1kHz 周期
if interp_count < interp_steps:
    t = interp_count / interp_steps
    v_cmd = v_start + t * (v_end - v_start)
    interp_count++
else:
    v_cmd = v_end
```

这样可产生更平滑的速度过渡，降低电机峻值电流和机械振动。

### 16.9 跟随超时看门狗

如果上层停止调用 updateFollowTarget，但未显式调用 stopFollowMode，电机会以最后一次计算的速度持续运行，非常危险。

必须实现看门狗机制：

1. 每次 updateFollowTarget 时记录时间戳。
2. 60Hz 规划线程每次检查：若距离上次 updateFollowTarget 超过 watchdog_timeout_ms，则开始减速停止。
3. 减速停止愿策略：按限制减速度将速度命令线性降到零。
4. 速度降为零后自动转入 Stopping 状态。
5. 记录告警日志。

---

## 17. 基础控制接口设计

建议对外提供以下接口：

```cpp
class ErobArmController {
public:
    bool initialize();
    bool shutdown();

    std::vector<AdapterInfo> scanAdapters();
    std::vector<MotorIdentity> scanMotorsOnAllAdapters();
    std::vector<MotorIdentity> rescan();

    bool connect(const std::string& adapter_name);
    bool bindAxis(int axis_id, const MotorIdentity& motor);

    bool enableAxis(int axis_id);
    bool disableAxis(int axis_id);
    bool resetFault(int axis_id);

    bool moveTo(int axis_id, double angle_deg, double velocity_deg_s);

    bool startFollowMode(int axis_id);
    bool updateFollowTarget(int axis_id, double angle_deg);
    bool stopFollowMode(int axis_id);

    AxisState getAxisState(int axis_id) const;
    std::array<AxisState, 3> getAllAxisStates() const;

    bool quickStopAxis(int axis_id);
    bool quickStopAll();
};
```

---

## 18. 安全策略设计

### 18.1 全模式运行时位置限位

所有电机在任何运动模式下均必须执行位置限位。默认初始限位值为 -130° 到 130°。位置限位分为两层：

**第一层：命令入口检查**

适用于所有模式。任何目标位置在进入控制器之前必须 clamp 到 [min_angle_deg, max_angle_deg]。

- PP 模式：target_position 超过限位时拒绝命令或截断到限位边界。
- CSV 随动模式：updateFollowTarget 中将目标角度 clamp 到限位范围内。
- CSP 模式（后续）：目标位置直接 clamp。

**第二层：周期线程运行时监控**

适用于 CSV、CST 等非位置直接控制的模式。在 1kHz 周期线程中持续监控 actual_angle，当接近限位时按比例削减同向命令，到达限位时强制归零（见 §16.7）。

各模式限位执行方式汇总：

| 模式 | 命令入口检查 | 运行时限位器 | 说明 |
|------|-----------|-----------|------|
| PP   | ✓ 拒绝或截断 | 可选（驱动内部执行） | 驱动器负责加减速，位置不会超调 |
| CSV  | ✓ clamp 目标 | ✓ 必须启用 | 速度模式无内置位置限制 |
| CSP  | ✓ clamp 目标 | 可选 | 位置直算模式，不易超调 |
| CST  | ✓ | ✓ 必须启用 | 力矩模式无内置位置限制 |

### 18.2 软限位配置

每个轴必须配置：

- min_angle_deg：默认 -130.0
- max_angle_deg：默认 130.0
- position_limit_margin_deg：默认 5.0（减速区宽度）

### 18.3 速度和加速度限幅

所有来自上层的速度请求都要做限幅。

### 18.4 故障策略

建议支持两种模式：

- 单轴故障仅停该轴
- 任意轴故障触发全臂急停

这项策略建议在配置文件中可选。

### 18.5 急停策略

急停应保证：

- 立即停止接收新的运动目标
- 随动模式速度命令迅速置零
- 必要时下发 quick stop 控制字

### 18.6 位置越限应急处理

如果实际位置已经超出软限位（例如惯性滑行或外力推动），系统应：

1. 立即止向超限方向发送任何命令。
2. 若在 CSV 模式，允许反向速度使其回退到限位内。
3. 若超出量超过安全余量（例如 > 2°），触发 quick stop。
4. 记录越限事件日志。
5. 可配置策略：“允许回退” vs “直接 quick stop”。

### 18.7 重扫保护

运动过程中不允许执行全网卡重扫。重扫只能在以下状态下执行：

- 系统未使能
- 或全轴已停止并进入安全状态

---

## 19. 通信与故障恢复设计

### 19.1 监测项

周期监测以下内容：

- WKC 是否达到 expectedWKC
- 从站状态是否为 OPERATIONAL
- 是否出现 SAFE_OP + ERROR
- 是否 lost
- DC 同步偏差是否异常

### 19.2 恢复策略

建议参考当前 demo 的恢复逻辑，但封装为类方法：

- acknowledge safe-op + error
- request back to operational
- reconfig slave
- recover slave

### 19.3 恢复分级

建议分三级：

- Level 1：短暂 WKC 波动，仅告警
- Level 2：单轴从站状态异常，尝试自动恢复
- Level 3：多次恢复失败，进入系统故障并停止运动

---

## 20. 配置文件设计

建议主配置文件使用 YAML，设备发现缓存使用 JSON。

### 20.1 主配置示例

```yaml
system:
  preferred_adapter: enp6s0
  ethercat_cycle_hz: 1000
  follow_control_hz: 60
  fault_policy: all_stop

axes:
  - logical_axis_id: 0
    joint_name: base
    motor_serial: EROB-A001
    counts_per_degree: 1456.3
    home_offset_count: 0
    min_angle_deg: -130.0
    max_angle_deg: 130.0
    max_velocity_deg_s: 180.0
    max_accel_deg_s2: 300.0
    max_decel_deg_s2: 300.0
    follow:
      kp: 8.0
      kd: 0.15
      deadband_deg: 0.05
      max_velocity_deg_s: 250.0
      max_accel_deg_s2: 600.0
      max_decel_deg_s2: 600.0
      target_filter_alpha: 0.4
      watchdog_timeout_ms: 50.0
      position_limit_margin_deg: 5.0

  - logical_axis_id: 1
    joint_name: shoulder
    motor_serial: EROB-A002
    counts_per_degree: 1820.0
    home_offset_count: 0
    min_angle_deg: -130.0
    max_angle_deg: 130.0
    max_velocity_deg_s: 150.0
    max_accel_deg_s2: 240.0
    max_decel_deg_s2: 240.0
    follow:
      kp: 7.5
      kd: 0.18
      deadband_deg: 0.05
      max_velocity_deg_s: 220.0
      max_accel_deg_s2: 550.0
      max_decel_deg_s2: 550.0
      target_filter_alpha: 0.4
      watchdog_timeout_ms: 50.0
      position_limit_margin_deg: 5.0

  - logical_axis_id: 2
    joint_name: elbow
    motor_serial: EROB-A003
    counts_per_degree: 1820.0
    home_offset_count: 0
    min_angle_deg: -130.0
    max_angle_deg: 130.0
    max_velocity_deg_s: 150.0
    max_accel_deg_s2: 240.0
    max_decel_deg_s2: 240.0
    follow:
      kp: 7.5
      kd: 0.18
      deadband_deg: 0.05
      max_velocity_deg_s: 220.0
      max_accel_deg_s2: 550.0
      max_decel_deg_s2: 550.0
      target_filter_alpha: 0.4
      watchdog_timeout_ms: 50.0
      position_limit_margin_deg: 5.0
```

---

## 21. 初始化流程设计

建议系统初始化流程如下：

1. 加载配置文件。
2. 扫描本机网卡。
3. 优先尝试 preferred adapter。
4. 扫描 EtherCAT 从站。
5. 识别 eRob 电机。
6. 对照配置文件完成轴绑定。
7. 为每个轴完成 PDO 映射。
8. 切换到 SAFE_OP。
9. 创建周期线程和监控线程。
10. 切换到 OPERATIONAL。
11. 将轴状态置为 ready。

---

## 22. 运行流程设计

### 22.1 正常周期流程

EtherCAT 周期线程每次循环执行：

1. 接收 processdata
2. 更新每个轴的实际状态（角度、速度、力矩）
3. 计算每轴的 near_positive_limit / near_negative_limit 标志
4. 读取命令双缓冲的 front buffer
5. 根据各轴模式生成发送 PDO
6. **执行运行时位置限位器**（见 §16.7 和 §18.1）：对 CSV/CST 模式的速度/力矩命令做最终安全截断
7. **执行速度和加速度限幅**
8. 下发 processdata
9. WKC 检查与统计

### 22.2 位置模式命令执行流程

1. 上层调用 moveTo
2. 命令写入 axis command
3. 周期线程检测到新命令
4. 若模式不对，先切到 PP
5. 写 profile 参数
6. 写 target_position
7. 发控制字触发新 set-point
8. 周期读取状态直至到位

### 22.3 随动模式命令执行流程

1. 上层调用 startFollowMode
2. 周期线程切换到 CSV
3. 上层不断调用 updateFollowTarget
4. 60 Hz 跟随线程读取最新目标和当前反馈
5. clamp 目标角度到限位范围内
6. 计算下一帧目标速度
7. 写入命令双缓冲 back buffer 并原子翻转
8. 1kHz 周期线程读取速度命令，做线性插值（见 §16.8）
9. 周期线程执行运行时位置限位器（见 §16.7）
10. 写入 PDO

---

## 23. 类之间关系说明

### 23.1 EthercatMasterSession

建议成员：

```cpp
class EthercatMasterSession {
public:
    bool scanAdapters(std::vector<AdapterInfo>* adapters);
    bool connect(const std::string& adapter_name);
    bool discoverMotors(std::vector<MotorIdentity>* motors);
    bool configurePdos();
    bool requestOperational();
    bool start();
    void stop();

    bool readAxisFeedback(uint16_t slave_index, TxPdoCommon* txpdo);
    bool writeAxisCommand(uint16_t slave_index, const void* rxpdo, size_t size);

    bool sdoWriteU8(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t value);
    bool sdoWriteU16(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t value);
    bool sdoWriteU32(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t value);

private:
    ecx_contextt context_;
    char iomap_[8192];
};
```

### 23.2 ErobAxis

```cpp
class ErobAxis {
public:
    explicit ErobAxis(AxisConfig config);

    bool enable(EthercatMasterSession* session);
    bool disable(EthercatMasterSession* session);
    bool resetFault(EthercatMasterSession* session);

    bool setProfilePositionTarget(double angle_deg, double velocity_deg_s);
    bool enterFollowMode();
    bool updateFollowTarget(double angle_deg);
    bool stopFollowMode();

    void updateFeedback(const TxPdoCommon& txpdo);
    void buildRxPdoForCycle(RxPdoUnified* pdo) const;
    void applyPositionLimiter(RxPdoUnified* pdo) const;

    bool isNearPositiveLimit() const;
    bool isNearNegativeLimit() const;
    AxisState getState() const;

private:
    AxisConfig config_;
    AxisState state_;
    AxisCommand command_;
    FollowParams follow_params_;
};
```

### 23.3 ErobArmController

```cpp
class ErobArmController {
public:
    bool initialize();
    bool loadConfig(const std::string& path);
    bool saveConfig(const std::string& path) const;

    std::vector<AdapterInfo> scanAdapters();
    std::vector<MotorIdentity> scanMotors();
    bool connectAndBind();

    bool enableAll();
    bool disableAll();
    bool quickStopAll();

    bool moveTo(int axis_id, double angle_deg, double velocity_deg_s);
    bool startFollowMode(int axis_id);
    bool updateFollowTarget(int axis_id, double angle_deg);
    bool stopFollowMode(int axis_id);

private:
    EthercatMasterSession master_;
    std::array<std::unique_ptr<ErobAxis>, 3> axes_;
};
```

---

## 24. 日志与诊断设计

### 24.1 日志分级

建议分为：

- ERROR
- WARN
- INFO
- DEBUG

### 24.2 实时线程日志限制

实时线程内只允许：

- 简单计数器统计
- 低频率摘要输出

不要在每个周期内打印详细日志。

### 24.3 状态快照

建议定期导出：

- 每轴位置
- 每轴速度
- 当前模式
- statusword
- WKC
- DC 偏差

---

## 25. 测试与验证计划

### 25.1 第一阶段测试

- 网卡扫描测试
- 单网卡多次连接测试
- 从站识别测试
- 轴绑定测试

### 25.2 第二阶段测试

- 单轴使能和失能测试
- 单轴故障恢复测试
- 三轴同时在线状态测试

### 25.3 第三阶段测试

- 单轴位置模式测试
- 不同速度参数下的位置模式测试
- 软限位测试
- 连续目标切换测试

### 25.4 第四阶段测试

- 单轴随动模式测试
- 60 Hz 外部目标更新测试
- 目标快速变化时的稳定性测试
- 长时间运行稳定性测试

### 25.5 系统级测试

- 掉线恢复测试
- WKC 异常测试
- 单轴故障触发全臂停机测试
- 多次启动停止测试

---

## 26. 分阶段实施计划

### 26.1 阶段一：通信层重构

目标：

- 将 SOEM 流程从 demo 中拆到 EthercatMasterSession
- 支持网卡扫描、连接、从站发现
- 支持 PDO 映射和基础状态切换

产出：

- EthercatMasterSession
- 发现结果导出
- demo 级连接验证程序

### 26.2 阶段二：单轴对象化

目标：

- 完成 ErobAxis
- 每轴独立状态、命令、反馈
- 使能、失能、清故障接口可用

产出：

- 单轴控制逻辑
- 三轴对象管理能力

### 26.3 阶段三：位置模式

目标：

- 实现 moveTo
- 实现软限位、单位换算、到位判定

产出：

- 单轴位置控制稳定版本

### 26.4 阶段四：随动模式

目标：

- 实现 CSV 跟随控制
- 60 Hz 规划线程稳定运行

产出：

- 随动模式可用版本

### 26.5 阶段五：系统增强

目标：

- 加入持久化配置
- 加入全局故障策略
- 加入更多诊断与测试工具

---

## 27. 性能设计要求

### 27.1 内存管理

- 所有运行时使用的缓冲区、数据结构必须在初始化阶段预分配，禁止在 1kHz 周期线程中执行任何堆内存分配。
- PDO 缓冲区、状态结构、命令结构均使用固定大小数组或嵌入式结构体。
- iomap 缓冲区分配 8192 字节，足够覆盖 3 个从站的混合 PDO 映射。

### 27.2 线程与调度

- 1kHz 周期线程必须设置 SCHED_FIFO 调度策略，优先级不低于 49。
- 周期线程应绑定到独立 CPU 核心（CPU affinity），避免与管理线程共享核心。
- 60Hz 规划线程优先级低于周期线程，但高于普通线程。
- 监控线程使用普通优先级即可。

### 27.3 数据对齐与缓存优化

每轴的 PDO 缓冲、状态结构和命令结构应使用 cache line 对齐，避免多轴之间产生 false sharing：

```cpp
struct alignas(64) PerAxisData {
    RxPdoUnified rx_pdo;
    TxPdoCommon tx_pdo;
    AxisState state;
    AxisCommand command;
};
```

当周期线程处理 3 个轴时，每个轴的数据独立不交叉，避免 CPU 缓存一致性协议的额外开销。

### 27.4 IO 路径优化

- 周期线程每次循环只调用一次 ec_send_processdata / ec_receive_processdata，不要在循环内做多次 SDO 读写。
- 模式切换、轮廓参数设置等 SDO 操作应在管理线程中执行，不占用周期线程时间。
- 若 PP 模式下的 profile velocity/acceleration/deceleration（0x6081/0x6083/0x6084）变化不频繁，应仅在参数变化时通过 SDO 写入一次，而非每次 moveTo 都写。

### 27.5 无锁后备方案

若平台不支持原子双缓冲（极少见），可退化为：

- pthread_spinlock（用户态自旋，不进内核）
- 保护范围仅限于单次 memcpy（不超过 64 字节）

---

## 28. 风险与应对

### 28.1 风险一：驱动对象字典与预期不一致

应对：

- 在实现前用 slaveinfo 或 SDO 读写确认实际对象。
- 将模式相关对象地址整理成统一配置。

### 28.2 风险二：轴唯一标识不稳定

应对：

- 尽量读取设备序列号。
- 若无法读取，则增加人工确认绑定流程。

### 28.3 风险三：实时线程周期抖动

应对：

- 使用实时内核。
- CPU 隔离和亲和性绑定。
- 避免实时线程内复杂逻辑。

### 28.4 风险四：随动参数不稳定导致振荡

应对：

- 从较保守的 kp、kd 起步。
- 先做单轴调参，再扩展到三轴。
- 增加死区、速度限幅、加速度限幅和目标滤波。

### 28.5 风险五：多轴拓扑改变导致绑定错误

应对：

- 启动时打印发现结果和绑定结果。
- 配置中保存 motor_serial。
- 启动时若匹配失败则拒绝直接运行。

---

## 29. 最终推荐实施结论

结合当前仓库现状，推荐最终方案如下：

1. 不在现有 demo 文件上继续直接叠加逻辑。
2. 新建可复用的 EthercatMasterSession、ErobAxis、ErobArmController 三层架构。
3. 第一阶段位置模式使用 PP。
4. 第一阶段随动模式使用 CSV，60 Hz 规划 + 1kHz 线性插值 + 运行时位置限位器。
5. 使用统一混合 PDO（RxPdoUnified），避免运行时重映射。
6. 周期线程与规划线程之间采用无锁双缓冲传递数据。
7. 每个轴必须独立维护：
   - 身份信息
   - 配置参数
   - 状态反馈
   - 当前命令
   - 模式与故障状态
8. 所有电机所有模式均执行位置限位，默认范围 -130° 到 130°。
9. 首次运行自动扫描全部网卡并记录设备，后续支持重扫更新。
10. 通过配置文件完成三轴逻辑绑定、零位偏移、软限位和跟随参数管理。
11. 从一开始就将故障恢复、急停、重扫保护和跟随看门狗纳入正式设计。

该方案兼顾当前仓库可复用能力、实现复杂度、可调试性和后续扩展性，是当前项目阶段最稳妥的落地路径。

---

## 30. 后续实现建议

建议按以下顺序继续开发：

1. 先实现 EthercatMasterSession 和扫描 demo。
2. 再实现 ErobAxis 与三轴绑定。
3. 先跑通位置模式。
4. 再实现 CSV 跟随模式。
5. 最后增加配置文件、诊断和系统级异常恢复。