# 智能分拣系统机构电机与运动参数配置指南 (configure.md)

本文档针对智能分拣系统（STM32F407 + FreeRTOS）的各个执行机构（CoreXY 步进电机、夹爪升降/开合舵机、旋转托盘舵机、旋转储物盘电机）的**运动范围**、**初始化状态**、**运动固定状态**以及所有**硬编码数据**进行全面梳理，并明确标注具体的修改文件路径与行号。

---

## 目录
1. [机构分配总览表](#1-机构分配总览表)
2. [CoreXY 双轴运动系统 (XY 轴)](#2-corexy-双轴运动系统-xy-轴)
3. [夹爪升降机构 (Z 轴舵机 1)](#3-夹爪升降机构-z-轴舵机-1)
4. [夹爪开合机构 (开合舵机 2)](#4-夹爪开合机构-开合舵机-2)
5. [旋转托盘机构 (舵机 3)](#5-旋转托盘机构-舵机-3)
6. [旋转储物盘机构 (大疆 DJI 3508 电机)](#6-旋转储物盘机构-大疆-dji-3508-电机)
7. [关键业务流程与固定点位速查表](#7-关键业务流程与固定点位速查表)
8. [参数调优与台架标定操作步骤](#8-参数调优与台架标定操作步骤)

---

## 1. 机构分配总览表

| 机构名称 | 驱动单元 / 通信总线 | 硬件引脚 / 资源 | 自由度 / 运动形式 | 额定工作行程 | 关键状态位置 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **XY 双轴运动平台** | 2× 张大头闭环步进 (ZDT X42/Y42) | CAN1 (PD0/PD1), 地址 1 & 2 | 平面 2 自由度 (CoreXY 联动) | 0 ~ 100 mm (X/Y) | 原点 (0,0), 放置区 (0,40) |
| **夹爪 Z 轴升降** | 模拟伺服舵机 (0~270°) | TIM1_CH1 (PE9, 50Hz PWM) | 单自由度垂直升降 | 0° ~ 90° | 抬起安全位 0°, 下潜抓取位 90° |
| **夹爪开合夹持** | 模拟伺服舵机 (0~270°) | TIM1_CH2 (PE11, 50Hz PWM) | 单自由度开合夹爪 | 0° ~ 90° | 完全张开 0°, 闭合抓取 90° |
| **旋转托盘** | 模拟伺服舵机 (0~270°) | TIM1_CH3 (PE13, 50Hz PWM) | 单自由度水平旋转 | 0° ~ 270° | 复位原点 0°, 自检测试角 0° |
| **旋转储物盘** | 大疆 DJI M3508 + 19:1 减速箱 | CAN1 (0x200 电流 / 0x201 反馈) | 360° 无限旋转 (6 分区) | 6 槽位 (每格 60°) | 槽位 0~5 对应 0°, 60°, 120°, 180°, 240°, 300° |

---

## 2. CoreXY 双轴运动系统 (XY 轴)

### 2.1 机械与动力学原理
XY 轴采用 CoreXY 传动拓扑，皮带轮由两个闭环步进电机驱动：
- 坐标变换公式：$A = X + Y$, $B = X - Y$
- 逆解变换公式：$X = \frac{A + B}{2}$, $Y = \frac{A - B}{2}$
- 步进细分与脉冲换算：
  - 同步带周长：20 齿 $\times$ 2mm 齿距 = 40.0 mm/圈
  - 整步电机：200 整步/圈 (1.8° 步距角)
  - 驱动器细分：16 细分
  - 脉冲当量：$\frac{200 \times 16}{40.0} = 80.0 \text{ 脉冲/mm}$
  - **0.1mm 分辨率网格**：$0.1\text{ mm} \times 80\text{ 脉冲/mm} = 8\text{ 脉冲}$（整脉冲，无累积截断误差）

### 2.2 硬编码数据与配置位置

| 配置宏定义名称 | 默认硬编码值 | 物理含义与工程单位 | 所在代码文件 | 行号参考 |
| :--- | :--- | :--- | :--- | :--- |
| `MS_XY_RESOLUTION_MM` | `0.1f` | XY 笛卡尔坐标分辨率网格 (mm) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L56 |
| `MS_XY_ARRIVE_TOLERANCE_MM`| `0.1f` | 物理实际到位判定容差 (mm) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L57 |
| `MS_XY_PULSES_PER_RES_UNIT`| `8U` | 0.1mm 分辨率对应的微步脉冲数 | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L59 |
| `MS_XY_ROTATION_MM` | `40.0f` | 电机转一圈皮带前进距离 (mm) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L65 |
| `MS_XY_FULL_STEPS` | `200U` | 1.8° 步进电机整步数 (steps) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L66 |
| `MS_XY_MICROSTEP` | `16U` | ZDT 步进驱动器硬件微步细分数 | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L67 |
| `MS_XY_MAX_VEL` | `300.0f` | XY 轴最大插补进给速度 (mm/s) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L45 |
| `MS_XY_MAX_ACCEL` | `3000.0f` | 最大规划加速度 (mm/s²) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L46 |
| `MS_XY_SCV` | `5.0f` | 直角拐弯允许通过速度 (mm/s) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L47 |
| `MS_XY_MOTOR_A_ADDR` | `1U` | CoreXY A 电机 CAN 扩展帧地址 | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L77 |
| `MS_XY_MOTOR_B_ADDR` | `2U` | CoreXY B 电机 CAN 扩展帧地址 | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L78 |
| `MS_XY_MOTOR_A_SIGN` | `(+1)` | A 电机运行正方向 (+1 或 -1) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L79 |
| `MS_XY_MOTOR_B_SIGN` | `(+1)` | B 电机运行正方向 (+1 或 -1) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L80 |
| `MS_XY_X_MIN` / `MAX` | `0.0f` / `100.0f`| 底层 X 轴软限位极值 (mm) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L102~103 |
| `MS_XY_Y_MIN` / `MAX` | `0.0f` / `100.0f`| 底层 Y 轴软限位极值 (mm) | `User/Module/Motion_SYS_XY/motion_sys_XY.h` | 约 L104~105 |
| `APP_HOME_X_MM` / `Y` | `0.0f` / `0.0f` | 业务待机原点位置 (mm) | `User/APP/app.h` | 约 L61~62 |
| `APP_PLACE_X_MM` / `Y` | `0.0f` / `40.0f`| 物块放置区目标位置 (mm) | `User/APP/app.h` | 约 L67~68 |
| `APP_XY_MIN_X_MM` / `MAX`| `0.0f` / `100.0f`| 业务层 X 轴安全行程 (mm) | `User/APP/app.h` | 约 L73~74 |
| `APP_XY_MIN_Y_MM` / `MAX`| `0.0f` / `100.0f`| 业务层 Y 轴安全行程 (mm) | `User/APP/app.h` | 约 L75~76 |
| `APP_FEED_RATE_MM_S` | `100.0f` | 前往取件/放置点进给速度 (mm/s)| `User/APP/app.h` | 约 L85 |
| `APP_FEED_RETURN_MM_S` | `120.0f` | 搬运完毕归位返回速度 (mm/s) | `User/APP/app.h` | 约 L90 |
| `HOST_MOVE_FEED` | `100.0f` | 上位机直控运动默认进给速度 (mm/s)| `User/Module/Communication/host_comm.h` | 约 L53 |
| `HOST_X_MIN_UM` / `MAX` | `0` / `100000` | 上位机 X 轴允许接收范围 (微米) | `User/Module/Communication/host_comm.h` | 约 L57~58 |
| `HOST_Y_MIN_UM` / `MAX` | `0` / `100000` | 上位机 Y 轴允许接收范围 (微米) | `User/Module/Communication/host_comm.h` | 约 L59~60 |

---

## 3. 夹爪升降机构 (Z 轴舵机 1)

### 3.1 机构工作状态
- **初始/安全状态**：上电时初始化为安全抬起位（0°），防止移动平台带着下沉的夹爪横扫工件造成机械碰撞。
- **下潜抓取状态**：到达取件或放置坐标并获得许可后，舵机下潜至 90° 执行工件抓取或释放。

### 3.2 硬编码数据与配置位置

| 配置宏定义名称 | 默认硬编码值 | 物理含义与工程单位 | 所在代码文件 | 行号参考 |
| :--- | :--- | :--- | :--- | :--- |
| `GRIP_Z_SERVO_CH` | `TIM_CHANNEL_1` | Z 轴升降舵机绑定的定时器通道 (PE9) | `User/Module/Grip/grip.h` | 约 L20 |
| `GRIP_Z_SAFE_DEG` | `0U` | Z 轴安全抬起角度 (度) | `User/Module/Grip/grip.h` | 约 L23 |
| `GRIP_Z_DOWN_DEG` | `90U` | Z 轴工作下潜抓取角度 (度) | `User/Module/Grip/grip.h` | 约 L24 |
| `GRIP_Z_MOVE_MS` | `500U` | Z 轴升降单程动作估算耗时 (ms) | `User/Module/Grip/grip.h` | 约 L33 |
| `APP_TIMEOUT_GRIP_MS` | `2000U` | 夹爪单步升降动作超时保护时限 (ms) | `User/APP/app.h` | 约 L130 |

---

## 4. 夹爪开合机构 (开合舵机 2)

### 4.1 机构工作状态
- **初始状态**：上电时初始化为完全张开态（0°）。
- **闭合抓取状态**：夹爪舵机向 90° 闭合，闭合角度根据力度百分比（0~100%）在线插值计算：
  $$\text{Target Angle} = \text{GRIP\_JAW\_OPEN\_DEG} + \frac{\text{Force}}{100} \times \text{GRIP\_JAW\_FORCE\_MAX\_DEG}$$
- 默认力度 100% 对应闭合到 90°。

### 4.2 硬编码数据与配置位置

| 配置宏定义名称 | 默认硬编码值 | 物理含义与工程单位 | 所在代码文件 | 行号参考 |
| :--- | :--- | :--- | :--- | :--- |
| `GRIP_JAW_SERVO_CH` | `TIM_CHANNEL_2` | 夹爪开合舵机绑定的定时器通道 (PE11)| `User/Module/Grip/grip.h` | 约 L21 |
| `GRIP_JAW_OPEN_DEG` | `0U` | 夹爪完全张开角度 (度) | `User/Module/Grip/grip.h` | 约 L27 |
| `GRIP_JAW_CLOSED_DEG`| `90U` | 夹爪闭合基准角度 (度) | `User/Module/Grip/grip.h` | 约 L28 |
| `GRIP_JAW_FORCE_MAX_DEG`| `90U` | 100% 夹持力度对应的旋转角行程 (度)| `User/Module/Grip/grip.h` | 约 L29 |
| `GRIP_JAW_MOVE_MS` | `500U` | 夹爪开合单程动作估算耗时 (ms) | `User/Module/Grip/grip.h` | 约 L34 |
| `APP_GRIP_FORCE_PCT` | `100U` | 业务层抓取时的夹持力度百分比 (0~100) | `User/APP/app.h` | 约 L95 |
| `HOST_GRIP_FORCE` | `100U` | 上位机通信协议默认夹持力度百分比 | `User/Module/Communication/host_comm.h` | 约 L84 |

---

## 5. 旋转托盘机构 (舵机 3)

### 5.1 机构工作状态
- **初始状态**：上电自检复位至 `0°`。
- **运动行程**：`0° ~ 270°` 全行程。开环状态下按 `4ms/deg` 的动态时间估算到位状态。

### 5.2 硬编码数据与配置位置

| 配置宏定义名称 | 默认硬编码值 | 物理含义与工程单位 | 所在代码文件 | 行号参考 |
| :--- | :--- | :--- | :--- | :--- |
| `TRAY_SERVO_CH` | `TIM_CHANNEL_3` | 旋转托盘舵机绑定的定时器通道 (PE13)| `User/Module/Tray/tray.h` | 约 L27 |
| `TRAY_HOME_DEG` | `0U` | 托盘开机复位基准原点 (度) | `User/Module/Tray/tray.h` | 约 L30 |
| `TRAY_MIN_DEG` | `0U` | 托盘允许旋转的最小限位角度 (度) | `User/Module/Tray/tray.h` | 约 L33 |
| `TRAY_MAX_DEG` | `270U` | 托盘允许旋转的最大限位角度 (度) | `User/Module/Tray/tray.h` | 约 L34 |
| `TRAY_MS_PER_DEG` | `4U` | 托盘每度旋转近似耗时 (ms/°) | `User/Module/Tray/tray.h` | 约 L71 |
| `TRAY_MOVE_MAX_MS` | `2000U` | 单次旋转动作的最大保护计时 (ms) | `User/Module/Tray/tray.h` | 约 L72 |
| `APP_TRAY_HOME_DEG` | `0U` | 业务主控层托盘复位目标角度 (度) | `User/APP/app.h` | 约 L100 |
| `APP_TRAY_CHECK_DEG`| `0U` | 开机自检旋转测试角度 (0=不转) | `User/APP/app.h` | 约 L105 |
| `APP_TIMEOUT_TRAY_MS`| `3000U` | 托盘动作业务级超时门限 (ms) | `User/APP/app.h` | 约 L114 |

---

## 6. 旋转储物盘机构 (大疆 DJI 3508 电机)

### 6.1 机构工作状态与槽位映射
储物盘由 DJI M3508 配合 19:1 减速机驱动，盘面等分为 6 个料格：
- 每个料格跨度：$360^\circ / 6 = 60.0^\circ$
- 槽位映射：
  - **槽位 0**：$0.0^\circ$（上电捕获初始位置作为 0 槽基准）
  - **槽位 1**：$60.0^\circ$
  - **槽位 2**：$120.0^\circ$
  - **槽位 3**：$180.0^\circ$
  - **槽位 4**：$240.0^\circ$
  - **槽位 5**：$300.0^\circ$
- **最短路径寻优**：顺时针与逆时针双向转动，单次转角最大不超过 $\pm 180^\circ$。
- **到位三要素判决**：$|\text{角度误差}| \le 6.0^\circ$ 且连续稳定保持 10 个控制周期（30ms）。

### 6.2 硬编码数据与配置位置

| 配置宏定义名称 | 默认硬编码值 | 物理含义与工程单位 | 所在代码文件 | 行号参考 |
| :--- | :--- | :--- | :--- | :--- |
| `STORAGE_MOTOR_CAN_ID` | `1U` | 3508 电机 CAN ID (0x201 接收 / 0x200 发送)| `User/Module/StorageTray/storage.h` | 约 L32 |
| `STORAGE_SLOT_NUM` | `6U` | 储物料盘等分槽位总数 | `User/Module/StorageTray/storage.h` | 约 L35 |
| `STORAGE_SLOT_ANGLE_DEG`| `60.0f` | 相邻两槽位之间的机械夹角 (度) | `User/Module/StorageTray/storage.h` | 约 L36 |
| `STORAGE_INIT_ANGLE_DEG`| `0.0f` | 上电捕获的 0 槽位起始角度 (度) | `User/Module/StorageTray/storage.h` | 约 L37 |
| `STORAGE_DIR_SIGN` | `(+1.0f)` | 槽位正向旋转方向 (+1.0 或 -1.0) | `User/Module/StorageTray/storage.h` | 约 L48 |
| `STORAGE_SPEED_RPM` | `9000.0f` | 电机转速上限 (减速前 r/min, 盘约 8.9rpm)| `User/Module/StorageTray/storage.h` | 约 L77 |
| `STORAGE_ARRIVE_DEG` | `6.0f` | 储物盘到位判决角度容差 (度) | `User/Module/StorageTray/storage.h` | 约 L80 |
| `STORAGE_HOLD_TICKS` | `10U` | 到位条件满足后持续保持拍数 (3ms/拍) | `User/Module/StorageTray/storage.h` | 约 L83 |
| `M3508_DEC` | `19` | 3508 外部减速箱减速比 | `User/Device/DJ_Motor/dji_3508.h` | 约 L35 |
| `DJ_CTRL_PERIOD` | `0.003f` | 3508 闭环控制周期 (3ms, 333.3Hz) | `User/Device/DJ_Motor/dji_3508.h` | 约 L42 |

---

## 7. 关键业务流程与固定点位速查表

在分拣全流程状态机（`User/APP/app.c`）中，机构各自由度在各阶段的动作协同状态定义如下：

```
上电自检 (APP_STATE_INIT)
  ├─ 托盘归位: 转至 APP_TRAY_HOME_DEG (0°)
  ├─ 夹爪归位: Z 轴抬起至 GRIP_Z_SAFE_DEG (0°), 爪部张开至 GRIP_JAW_OPEN_DEG (0°)
  └─ XY 轴归位: 建立原点 (APP_HOME_X_MM=0, APP_HOME_Y_MM=0)
       ↓
待机就绪 (APP_STATE_IDLE) ── 接收上位机坐标任务 (target_x, target_y)
       ↓
移动前往取件位 (APP_STATE_MOVING_TO_TARGET)
  ├─ XY 运动至: (target_x, target_y), 速度 APP_FEED_RATE_MM_S (100 mm/s)
  └─ 实际到位判定: 满足 MS_XY_ARRIVE_TOLERANCE_MM (0.1mm) 容差确认
       ↓
等待整机上位机许可信号 (APP_STATE_WAIT_PERMISSION) ── 等待 HOST_FLAG_PERMIT
       ↓
夹爪下潜 (APP_STATE_GRIP_DOWN) ── Z 轴下潜至 GRIP_Z_DOWN_DEG (90°)
       ↓
夹爪闭合抓取 (APP_STATE_GRIP_CLOSE) ── 闭合至目标力度 APP_GRIP_FORCE_PCT (100%)
       ↓
抓取校验 (APP_STATE_GRIP_CHECK) ── 成功进入抬升, 失败进入善后 (APP_STATE_GRIP_FAIL)
       ↓
夹爪安全抬升 (APP_STATE_GRIP_UP) ── Z 轴抬升至 GRIP_Z_SAFE_DEG (0°)
       ↓
携物移动前往放置区 (APP_STATE_MOVING_TO_PLACE)
  ├─ XY 运动至放置区: (APP_PLACE_X_MM=0.0, APP_PLACE_Y_MM=40.0)
  └─ 速度: APP_FEED_RATE_MM_S (100 mm/s)
       ↓
放置区释放 (APP_STATE_RELEASE) ── 夹爪张开至 GRIP_JAW_OPEN_DEG (0°)
       ↓
归位返回待机位 (APP_STATE_RETURN_HOME)
  ├─ XY 轴运动至原点: (APP_HOME_X_MM=0.0, APP_HOME_Y_MM=0.0)
  └─ 返航速度: APP_FEED_RETURN_MM_S (120 mm/s)
       ↓
回到待机状态 (APP_STATE_IDLE)
```

---

## 8. 参数调优与台架标定操作步骤

### 8.1 调整放置区或待机原点位置
1. 打开 `User/APP/app.h`；
2. 找到 `APP_PLACE_X_MM` 与 `APP_PLACE_Y_MM`（约第 67~68 行），修改为实际机械放置槽坐标（如放置在 X=20.0mm, Y=50.0mm）；
3. 确保坐标数值为 `0.1mm` 的整数倍（如 `20.0f`，`50.5f`）。

### 8.2 调整 XY 运动范围与软限位
1. 打开 `User/APP/app.h`；
2. 修改 `APP_XY_MAX_X_MM` 与 `APP_XY_MAX_Y_MM`（约第 74 与 76 行）；
3. 同步修改 `User/Module/Motion_SYS_XY/motion_sys_XY.h` 中的 `MS_XY_X_MAX` 与 `MS_XY_Y_MAX`；
4. 同步修改 `User/Module/Communication/host_comm.h` 中的 `HOST_X_MAX_UM` 与 `HOST_Y_MAX_UM`（微米，需乘以 1000）。

### 8.3 调整夹爪高度与闭合行程
1. 打开 `User/Module/Grip/grip.h`；
2. 若物块较厚需要提高抓取高度：调小 `GRIP_Z_DOWN_DEG`（如从 90° 调整为 75°）；
3. 若夹爪抓取松紧度需要标定：
   - 调整 `GRIP_JAW_CLOSED_DEG` 设定完全紧闭的角度；
   - 或在 `User/APP/app.h` 中调整 `APP_GRIP_FORCE_PCT`（0~100）。

### 8.4 储物盘转向与减速比标定
1. 打开 `User/Module/StorageTray/storage.h`；
2. 若实机装配发现槽位 1 旋转方向反向：将 `STORAGE_DIR_SIGN` 从 `(+1.0f)` 修改为 `(-1.0f)`；
3. 若更换了减速箱（如 36:1 或 2006 电机）：在 `User/Device/DJ_Motor/dji_3508.h` 中修改 `M3508_DEC`。
