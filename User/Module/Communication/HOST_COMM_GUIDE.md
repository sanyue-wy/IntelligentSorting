# 智能分拣系统上位机通信协议与开发指南

本文档面向上位机软件开发人员（Python / C++ / C# / ROS / 工业视觉系统），详细说明与 STM32 固件之间的 **USB CDC 虚拟串口二进制通信协议**、**典型业务交互流程** 以及 **开箱即用的上位机端参考代码**。

---

## 1. 物理层与传输规约

* **物理接口**：USB CDC (Virtual COM Port, 虚拟串口，全速 12 Mbps)。
* **波特率参数**：虚拟串口波特率自适应（默认推荐 `115200, 8-N-1`）。
* **帧格式**：**定长 32 字节二进制帧**（上下行均为 32 字节）。
* **字节序**：多字节整型（`uint16_t`, `int32_t`, `uint32_t`）均采用 **小端模式 (Little-Endian，低字节在前)**。
* **坐标单位**：协议中坐标均以 **微米 ($\mu\text{m}$, 0.001 mm)** 为单位（`int32_t`），换算关系：$1\text{ mm} = 1000\ \mu\text{m}$。

---

## 2. 协议帧结构定义

### 2.1 下行命令帧 (Host $\rightarrow$ STM32，固定 32 字节)

| 字节偏移 | 字段名称 | 类型 | 说明与取值 |
| :--- | :--- | :--- | :--- |
| `[0..1]` | `SOF` | `uint8_t[2]` | 固定同步头：`0xAA 0x55` |
| `[2]` | `VER` | `uint8_t` | 协议版本号，固定为 `0x01` |
| `[3]` | `TYPE` | `uint8_t` | 帧类型：`0x01` (命令帧) |
| `[4]` | `FLAGS` | `uint8_t` | **控制命令标志位集合 (位掩码)**，见下表 |
| `[5..6]` | `SEQ` | `uint16_t` | 下行命令流水号，单调递增 (用于重发去重与 ACK) |
| `[7..10]` | `TASK_ID` | `uint32_t` | 任务流水号，每次新分拣任务递增 |
| `[11..14]` | `TARGET_X` | `int32_t` | 目标 X 坐标 (单位: $\mu\text{m}$，例如 $40\text{mm} = 40000$) |
| `[15..18]` | `TARGET_Y` | `int32_t` | 目标 Y 坐标 (单位: $\mu\text{m}$，例如 $25.5\text{mm} = 25500$) |
| `[19]` | `GRIP_CMD` | `uint8_t` | 夹爪直接命令：`0=保持`, `1=张开`, `2=闭合` |
| `[20..21]` | `TIMEOUT` | `uint16_t` | 命令超时时限 (ms)，`0` 表示不检查超时 |
| `[22..31]` | `RESERVED` | `uint8_t[10]`| 预留，必须全填 `0x00` |

#### 下行 `FLAGS` 位掩码说明
* `0x01` (`HOST_FLAG_MOTION`)：**运动有效**。包含目标 X,Y 坐标，通知平台移动到该位置；
* `0x02` (`HOST_FLAG_GRIP`)：夹爪动作有效；
* `0x04` (`HOST_FLAG_ESTOP`)：**急停**。机构立刻刹停并锁存，直至发送 CLEAR；
* `0x08` (`HOST_FLAG_CLEAR`)：**清除报警/复位**。清除错误和急停锁存，恢复待机；
* `0x10` (`HOST_FLAG_CANCEL`)：**取消任务**。中止当前在途任务并安全复位；
* `0x20` (`HOST_FLAG_PERMIT`)：**抓取许可信号**。到位后上位机授权允许下潜抓取。

---

### 2.2 上行状态帧 (STM32 $\rightarrow$ Host，固定 32 字节)

下位机在**事件跳变**（到位、许可响应、抓取结果、报警）或**坐标移动**（限频 20Hz）时主动上报：

| 字节偏移 | 字段名称 | 类型 | 说明与取值 |
| :--- | :--- | :--- | :--- |
| `[0..1]` | `SOF` | `uint8_t[2]` | 固定同步头：`0xAA 0x55` |
| `[2]` | `VER` | `uint8_t` | 协议版本号，固定为 `0x01` |
| `[3]` | `TYPE` | `uint8_t` | 帧类型：`0x81` (状态上报帧) |
| `[4]` | `FLAGS` | `uint8_t` | **系统运行标志位集合**，见下文 |
| `[5..6]` | `SEQ` | `uint16_t` | 上行帧流水号 (每发送成功一帧递增) |
| `[7..10]` | `TASK_ID` | `uint32_t` | 当前正在执行或最近完成的任务 ID |
| `[11..14]` | `CUR_X` | `int32_t` | 当前实际闭环 X 坐标 (单位: $\mu\text{m}$) |
| `[15..18]` | `CUR_Y` | `int32_t` | 当前实际闭环 Y 坐标 (单位: $\mu\text{m}$) |
| `[19]` | `STATE` | `uint8_t` | 系统主状态枚举，见下文 |
| `[20]` | `GRIPPER` | `uint8_t` | 夹爪当前状态，见下文 |
| `[21..22]` | `ERROR` | `uint16_t` | 错误掩码（`0x0000` 表示无故障），见下文 |
| `[23..24]` | `ACK_SEQ` | `uint16_t` | 最近成功受理的下行 `SEQ` |
| `[25]` | `PROGRESS` | `uint8_t` | 动作执行进度百分比 ($0\sim100$) |
| `[26]` | `RES_FLAGS` | `uint8_t` | **任务结果标志位**，见下文 |
| `[27]` | `RES_STAGE` | `uint8_t` | 故障/中止发生的阶段码，见下文 |
| `[28..31]` | `RESERVED` | `uint8_t[4]` | 预留，全 `0x00` |

#### 上行枚举与状态值说明
* **上行 `FLAGS` (偏移 4)**：
  - `0x01` (`BUSY`)：机构运动或动作中；
  - `0x02` (`ALARM`)：存在未清除的报警；
  - `0x04` (`ESTOP`)：处于急停锁存状态；
  - `0x08` (`RESULT`)：任务结果字段有效；
  - `0x10` (`TASK_DONE`)：任务刚完成事件跳变；
* **主状态 `STATE` (偏移 19)**：
  - `0`: `IDLE` (待机空闲)
  - `2`: `RUNNING` (机构动作执行中)
  - `4`: `ALARM` (故障报警锁存)
  - `5`: `ESTOP` (急停锁存)
  - `6`: `TASK_DONE` (任务刚完成)
* **任务结果 `RES_FLAGS` (偏移 26)**：
  - `0x01` (`HOST_RES_DONE`)：任务已完结
  - `0x02` (`HOST_RES_OK`)：**任务成功完成**
  - `0x04` (`HOST_RES_PICK`)：**已到达取件点**（等待许可阶段）
  - `0x80` (`HOST_RES_ABORT`)：任务中止（急停/超时/取消）
* **错误码 `ERROR` (偏移 21..22)**：
  - `0x0002`: `HOST_ERR_PARAM` (坐标超出软限位)
  - `0x0008`: `HOST_ERR_TIMEOUT` (动作或等待许可超时)
  - `0x0020`: `HOST_ERR_GRIPFAIL` (**抓取失败，未夹到工件**)
  - `0x0100`: `HOST_ERR_ESTOP` (急停生效)
  - `0x0200`: `HOST_ERR_INTERNAL` (底层运动驱动故障)
  - `0x0800`: `HOST_ERR_FOREIGN_OBJ` (异物检测报警)

---

## 3. 标准业务交互流程与时序

```
上位机 (Host PC / 视觉)                           下位机 (STM32 分拣固件)
       │                                                 │
  [上电启动]                                        [开机自检完成]
       │ ── 查询状态 ─────────────────────────────────► │
       │ ◄─ 上报 STATE=IDLE (待机空闲) ──────────────── │
       │                                                 │
【1. 任务下发】                                          │
       │ ── 下发目标坐标 (FLAG_MOTION, X, Y, TASK_ID) ──►│
       │ ◄─ ACK_SEQ 确认, STATE=RUNNING ─────────────── │ (XY 轴插补前往目标点)
       │ ◄─ 周期位置上报 (CUR_X, CUR_Y) ──────────────── │
       │                                                 │
【2. 到位与权限校验】                                    │
       │ ◄─ RES_FLAGS 包含 RES_PICK (已到达取件位) ───── │ (平台停稳在工件正上方)
       │                                                 │ (等待上位机 PERMIT 信号)
       │ (上位机视觉/整机二次确认安全)                   │
       │ ── 下发抓取许可 (FLAG_PERMIT, TASK_ID) ────────►│
       │                                                 │ (夹爪下潜 -> 闭合抓取)
       │                                                 │
【3A. 抓取成功分支】                                     │
       │                                                 │ (Z抬起 -> 移动到放置区 ->
       │                                                 │  张开释放 -> 归位到原点)
       │ ◄─ RES_FLAGS=(DONE | OK), STATE=IDLE ────────── │ (整机任务圆满成功)
       │                                                 │
【3B. 抓取失败分支】                                     │
       │                                                 │ (判定抓空 -> Z抬起)
       │ ◄─ RES_FLAGS=DONE (无OK), ERR=GRIPFAIL ─────────│ (上报抓取失败, 原地待命)
       │                                                 │
       │ (上位机重新识别并下发新坐标)                    │
       │ ── 下发新坐标 (FLAG_MOTION, X_new, Y_new) ─────►│ (重新开始寻位)
```

---

## 4. 上位机 Python 参考驱动代码

以下为完整的、基于 `pyserial` 的上位机驱动类与自动化分拣测试示例：

```python
import time
import struct
import serial

class IntelligentSortingHost:
    SOF = b'\xAA\x55'
    VER = 0x01
    TYPE_CMD = 0x01
    TYPE_STATUS = 0x81

    # 下行控制 FLAGS
    FLAG_MOTION = 0x01
    FLAG_GRIP   = 0x02
    FLAG_ESTOP  = 0x04
    FLAG_CLEAR  = 0x08
    FLAG_CANCEL = 0x10
    FLAG_PERMIT = 0x20

    # 上行状态与结果
    RES_DONE  = 0x01
    RES_OK    = 0x02
    RES_PICK  = 0x04
    ERR_GRIPFAIL = 0x0020

    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.seq = 1

    def send_cmd(self, flags: int, task_id: int, x_mm: float = 0.0, y_mm: float = 0.0,
                 grip_cmd: int = 0, timeout_ms: int = 0):
        """构造并发送 32 字节下行二进制帧"""
        x_um = int(round(x_mm * 1000.0))
        y_um = int(round(y_mm * 1000.0))
        # 格式: 2B SOF, 1B VER, 1B TYPE, 1B FLAGS, 2B SEQ, 4B TASK_ID, 4B X, 4B Y, 1B GRIP, 2B TIMEOUT, 10B 0
        payload = struct.pack('<2sBBBHiiBH10s',
                              self.SOF, self.VER, self.TYPE_CMD, flags,
                              self.seq, task_id, x_um, y_um, grip_cmd, timeout_ms, b'\x00' * 10)
        self.seq = (self.seq + 1) & 0xFFFF
        self.ser.write(payload)

    def read_status(self):
        """读取并解析 32 字节上行状态帧"""
        if self.ser.in_waiting < 32:
            return None
        # 对齐帧头
        header = self.ser.read(2)
        while header != self.SOF:
            if not header or len(header) < 2:
                return None
            header = header[1:] + self.ser.read(1)

        body = self.ser.read(30)
        if len(body) != 30:
            return None

        data = header + body
        # 解析 32 字节字段
        sof, ver, msg_type, flags, seq, task_id, x_um, y_um, state, gripper, err, ack, progress, res_flags, res_stage = \
            struct.unpack('<2sBBBHiiBBHHBBB', data[:28])

        return {
            'flags': flags,
            'seq': seq,
            'task_id': task_id,
            'x_mm': x_um / 1000.0,
            'y_mm': y_um / 1000.0,
            'state': state,
            'gripper': gripper,
            'error': err,
            'ack': ack,
            'progress': progress,
            'res_flags': res_flags,
            'res_stage': res_stage
        }

    def execute_sorting_task(self, task_id: int, target_x_mm: float, target_y_mm: float, timeout_s: float = 20.0):
        """执行完整分拣闭环流程"""
        print(f"\n[Host] >>> 发起分拣任务 {task_id}: 目标坐标 ({target_x_mm:.2f}, {target_y_mm:.2f}) mm")
        # 1. 下发移动指令
        self.send_cmd(flags=self.FLAG_MOTION, task_id=task_id, x_mm=target_x_mm, y_mm=target_y_mm)
        
        start_t = time.time()
        permit_sent = False

        while time.time() - start_t < timeout_s:
            st = self.read_status()
            if not st:
                time.sleep(0.01)
                continue

            # 2. 检测是否到达目标点 (等待许可阶段)
            if (st['res_flags'] & self.RES_PICK) and not permit_sent:
                print(f"[Host] 平台已到达目标点 ({st['x_mm']:.2f}, {st['y_mm']:.2f}) mm! 视觉校验通过，下发 PERMIT 许可信号...")
                self.send_cmd(flags=self.FLAG_PERMIT, task_id=task_id)
                permit_sent = True

            # 3. 检查任务结果
            if st['res_flags'] & self.RES_DONE:
                if st['res_flags'] & self.RES_OK:
                    print(f"[Host] [SUCCESS] 任务 {task_id} 圆满成功! 物块已投放并归位。")
                    return True
                else:
                    if st['error'] & self.ERR_GRIPFAIL or st['res_stage'] == 2:
                        print(f"[Host] [WARN] 抓取失败 (未夹取到工件)! 机构已安全复位待命，准备重新识别下发新坐标...")
                        return False
                    else:
                        print(f"[Host] [FAIL] 任务异常终止! 错误码: 0x{st['error']:04X}, 阶段: {st['res_stage']}")
                        return False

            time.sleep(0.01)

        print("[Host] 任务超时未响应!")
        return False


if __name__ == '__main__':
    # 示例用法 (修改为实际的串口号)
    host = IntelligentSortingHost(port='COM3')
    
    # 清除任何历史残留报警
    host.send_cmd(flags=IntelligentSortingHost.FLAG_CLEAR, task_id=0)
    time.sleep(0.1)

    # 发起一次分拣测试 (取件坐标: X=40.0mm, Y=20.0mm)
    success = host.execute_sorting_task(task_id=101, target_x_mm=40.0, target_y_mm=20.0)
```

---

## 5. 常见异常排查指南

| 现象 | 上位机检测指标 | 推荐善后处置 |
| :--- | :--- | :--- |
| **坐标超限被拒** | 发送指令后未推进 `ACK_SEQ`，或收到 `ERROR = 0x0002` (`PARAM`) | 检查下发坐标是否在软限位 $[0, 100]\text{mm}$ 内 |
| **平台停在工件上方不动** | 收到 `RES_FLAGS & 0x04` (`RES_PICK`)，夹爪不下潜 | 正常等待门禁，上位机需发送 `FLAG_PERMIT` 才能触发抓取 |
| **抓空 / 抓取失败** | `RES_FLAGS & 0x01` 且无 `0x02`，`ERROR = 0x0020` | 机械臂已自动抬起安全待命，视觉重新计算工件中心并下发新坐标 |
| **急停锁存** | `FLAGS & 0x04` (`ESTOP`)，拒绝所有移动指令 | 排除现场险情后，上位机发送 `FLAG_CLEAR` (`0x08`) 即可解除锁存 |
| **异物检测报警** | 开机后收到 `ERROR = 0x0800` (`FOREIGN_OBJ`) | 检查转盘或物料区是否有非预期遮挡，清理后下发 `FLAG_CLEAR` |
