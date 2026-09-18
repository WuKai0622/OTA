# STM32F407 以太网 / 蓝牙双通道 IAP 在线固件升级系统

基于 **STM32F407ZGTx + LAN8720A** 的 Bootloader，支持经 **以太网(TCP)** 与 **蓝牙(HC-05 透传)** 两条链路远程升级应用固件 —— **不拆机、可远程、掉电不变砖**。

![platform](https://img.shields.io/badge/MCU-STM32F407ZGTx-blue)
![stack](https://img.shields.io/badge/TCP%2FIP-lwIP%202.1.2%20(NO__SYS%3D1)-green)
![phy](https://img.shields.io/badge/PHY-LAN8720A%20%2B%20RMII-orange)
![ide](https://img.shields.io/badge/IDE-Keil%20MDK-red)
![host](https://img.shields.io/badge/host-Python%20%2B%20PySide6-yellow)

---

## 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [Flash 分区](#flash-分区)
- [帧协议](#帧协议)
- [目录结构](#目录结构)
- [快速开始](#快速开始)
- [上位机用法](#上位机用法)
- [关键设计点](#关键设计点)
- [已知限制](#已知限制)
- [技术栈](#技术栈)

---

## 项目简介

产品交付后常需现场更新固件。传统方式依赖 **SWD 仿真器拆机烧录**：需要开壳、接线、带电脑，成本高且无法远程。

本项目的目标是做一套 **Bootloader + 上位机工具**，让固件升级变成"连上就能刷"：

- **不拆机** —— 通过设备已有的通信链路（网口 / 蓝牙）升级
- **可远程** —— 网口支持 TCP 直连，蓝牙支持笔记本/手机就地升级
- **掉电不变砖** —— A/B 双分区 + 健康标志，任意时刻断电复位都能回到可用固件
- **弱链路可用** —— 逐帧确认 + 超时重传 + 断点续传，丢包不导致整包重来

**双通道的价值**：现场有网线就用网口（快）；手边只有笔记本蓝牙就用蓝牙（免布线）。**两条链路共用同一套协议状态机**，上位机侧只需切换传输层。

---

## 功能特性

| 特性 | 说明 |
|---|---|
| **双通道统一协议** | 同一套帧协议状态机跑在 UART(USART3) 与 ETH(TCP) 上，与物理通道完全解耦 |
| **A/B 双分区 + 健康标志** | 新固件先落下载区，校验通过才搬入运行区；掉电可回退，不由半成品固件启动 |
| **逐帧 stop-and-wait** | 每帧独立 ACK，丢帧/坏帧只重发单帧，不必整包重传 |
| **幂等重传 / 断点续传** | 每帧携带起始偏移，已提交帧直接回 ACK 不重写 Flash；断线后可原偏移续传 |
| **以太网零拷贝接收** | 基于 lwIP 私有内存池合并 pbuf 与 DMA 缓冲区，每帧省去约 1KB 内存拷贝 |
| **CRC16-MODBUS 校验** | 覆盖 `[CMD..RSV]`，上位机与 boot 两侧查表逐值相同 |
| **帧长度越界防护** | `LEN` 超限直接丢弃重扫，防止缓冲区溢出 |
| **LCD 进度显示** | FSMC 驱动，实时显示已写入字节数 |
| **Flash 自检** | 擦写读回比对（使用预留区末尾扇区） |
| **配套上位机** | 无界面 CLI + PySide6 图形界面，内置坏帧注入 / 越界伪造 / CRC 自检等测试开关 |

---

## 系统架构

### 数据流（以太网通道）

```
        上位机 PC (TCP client)
              │
              │  网线：差分信号 + 4B/5B + MLT-3 编码
              ▼
        ┌───────────┐
        │   RJ45    │
        └─────┬─────┘
              ▼
     ┌──────────────────┐
     │  LAN8720A  (PHY) │  差分收发 · 编解码 · 时钟恢复 · 自协商
     └────────┬─────────┘
              │  RMII（数字信号：REF_CLK / TXD0,1 / RXD0,1 / TX_EN / CRS_DV）
              ▼
     ┌──────────────────┐
     │ STM32F407 ETH 外设│  = MAC：前导码检测 · MAC 过滤 · CRC(FCS) · DMA
     └────────┬─────────┘
              │  DMA 直写（零拷贝：数据不经 CPU 搬运）
              ▼
     RxBuff_t.buff[1536] ──┐
                           │  pbuf 直接坐落于同一块内存头部
                           ▼
     ┌──────────────────────────────────┐
     │  lwIP 2.1.2 (NO_SYS=1，主循环轮询) │
     └────────┬─────────────────────────┘
              ▼  ota_recv_cb
     ┌──────────────────┐
     │  Bootloader_Task │
     └────────┬─────────┘
              │  ① Bootloader_RingWrite  (pbuf → 环形缓冲)   ← 拷贝 1
              ▼
     ┌──────────────────┐
     │   接收环形缓冲    │
     └────────┬─────────┘
              │  ② RingRead               (环 → tmp[256])    ← 拷贝 2
              ▼
     ┌──────────────────┐
     │    Proto_Feed    │  帧同步 · CRC 校验 · 状态机
     └────────┬─────────┘
              │  ③ g_frm[g_frm_len++]     (tmp → 帧缓冲)      ← 拷贝 3
              ▼
     ┌──────────────────┐
     │  Flash_ProgramBytes │  字编程，无整块 memcpy
     └────────┬─────────┘
              ▼
        下载区 A (0x08060000)
```

**整条路径只有 3 次真实内存拷贝**（都在应用层），DMA 与协议栈各层均不拷贝数据 —— 详见 [关键设计点](#关键设计点)。

### 双通道汇流

```
                    ┌──────────────┐
   HC-05 蓝牙 ──────►│  USART3 中断 │──┐
   (19200 8N1)       └──────────────┘  │   临界区保护
                                       ├──► 同一个接收环形缓冲 ──► Proto_Feed()
                    ┌──────────────┐  │        （单一状态机）
   以太网 TCP ──────►│ lwIP 主循环  │──┘
                    │   轮询       │
                    └──────────────┘
```

> ⚠️ **ETH 中断在本工程中未使能** —— 以太网收包完全靠主循环轮询 `MX_LWIP_Process()`。
> 而 USART3 是中断驱动的，所以两者访问共享环形缓冲时必须用临界区保护
> （`__get_PRIMASK` / `__disable_irq` / `__set_PRIMASK`）。

---

## Flash 分区

目标芯片 STM32F407ZGT6，1MB 单 bank。**F4 的擦除单位是"扇区"，且大小不等**：
`S0–S3 = 16KB | S4 = 64KB | S5–S11 = 128KB`。

```
   0x08000000 ┌──────────────┐
              │  boot        │ 128KB   S0–S4     ← Bootloader 自身
   0x08020000 ├──────────────┤
              │  标志区       │ 128KB   S5        ← 状态记录 + 健康字
   0x08040000 ├──────────────┤
              │  run         │ 128KB   S6        ← 当前运行的 APP
   0x08060000 ├──────────────┤
              │  A 下载区     │ 128KB   S7        ← OTA 目标区
   0x08080000 ├──────────────┤
              │  B 备份区     │ 128KB   S8        ← 旧固件备份
   0x080A0000 ├──────────────┤
              │  预留         │ 384KB   S9–S11    ← S11 末尾用于 Flash 自检
   0x08100000 └──────────────┘
```

| 宏 | 地址 | 大小 | 说明 |
|---|---|---|---|
| `BL_BASE` | `0x08000000` | 128KB | Bootloader（S0–S4） |
| `PARAM_ADDR` | `0x08020000` | 128KB | 标志区（S5） |
| `APP_RUN_ADDR` | `0x08040000` | 128KB | APP 运行区（S6） |
| `APP_DL_ADDR` | `0x08060000` | 128KB | 下载区 A（S7） |
| `APP_BK_ADDR` | `0x08080000` | 128KB | 备份区 B（S8） |
| `APP_MAX_SIZE` | — | 128KB | 固件上限，**须与上位机 `ota_proto.APP_MAX_SIZE` 一致** |

> **⚠️ Bootloader 必须独占 S0–S4 共 128KB，不能只占 64KB。**
> 本工程 boot 镜像实测 **77.18KB**，代码铺到 `0x08013493`。若按 64KB 划分、把标志区放在 S4
> (`0x08010000`)，第一次 `Flags_Save()` 擦 S4 就会连带擦掉 boot 自己在
> `0x08010000–0x08013493` 的 13.2KB 代码，导致 HardFault + 重启黑屏。
> **链接脚本必须同步限制 IROM1 Size = 0x20000**，否则分区覆盖检查形同虚设。

---

## 帧协议

### 帧结构

```
┌────────┬────────┬────────┬──────────┬────────┬─────────┐
│ HEAD   │  CMD   │  LEN   │  DATA    │  RSV   │  CRC16  │
│ 2 字节 │ 2 字节 │ 2 字节 │  N 字节  │ 4 字节 │ 2 字节  │
│ A5 5A  │  LE    │  LE    │ N ≤ 1024 │  LE    │  LE     │
└────────┴────────┴────────┴──────────┴────────┴─────────┘
   ▲                                            ▲
   └── 不参与 CRC                      CRC16-MODBUS 覆盖 [CMD..RSV]
                                       共 8 + LEN 字节，init = 0xFFFF
```

**帧总长 = 12 + N**（含包头）。CRC 多项式 `0xA001`，查表实现，两侧表值逐项相同。

### 命令码

| 命令 | 值 | 方向 | DATA | RSV |
|---|---|---|---|---|
| `CMD_OTA_START` | `0x0001` | PC → boot | 固件总长 (u32) | 目标区（0 = A） |
| `CMD_OTA_DATA` | `0x0002` | PC → boot | 固件块 (≤1024B) | **本帧起始偏移 (u32)** |
| `CMD_OTA_END` | `0x0003` | PC → boot | 固件总长 (u32) | — |
| `CMD_RSP_ACK` | `0x0100` | boot → PC | 状态码 (1B) | 同请求帧的 RSV |

状态码：`RSP_OK = 0` / `RSP_ERR = 1`

### 升级流程

```
① OTA_START(总长 L, 区 A)
      boot: 校验 L → 擦除整个下载区（典型 1s，最坏 4s）→ ACK(OK)
      ⚠️ 擦除期间 Flash 总线 stall、CPU 冻结，上位机超时必须 ≥ 8s

② OTA_DATA(off, ≤1024B)   ← 逐帧，每帧等本偏移的 ACK(OK) 才发下一帧
      boot: off == 已写总数  → 正常写入   → ACK(OK)
            off + n ≤ 已写总数 → 重复帧，不重写 → ACK(OK)   ★ 幂等
            其他              → 错位缺帧   → ACK(ERR)

③ OTA_END(总长 L)
      boot: 累计字节 == L → ACK(OK) → 搬运至运行区 → 跳转
```

**② 的幂等分支是断点续传的基础**：ACK 在回程丢失、或连接中断后重连，上位机从原偏移重发，
设备端识别出"这帧已经提交过"直接确认、不重复写 Flash —— 于是**续传天然成立，且不需要额外的查询命令**。

---

## 目录结构

```
OTA_ETH/
├── Core/                       ← 应用核心（本项目的主要工作）
│   ├── Inc/
│   │   ├── Bootloader.h            启动流程、接收环、任务调度
│   │   ├── protocol.h              ★ 帧协议定义（与通道解耦）
│   │   ├── flash_if.h              ★ Flash 分区表 + 扇区操作
│   │   ├── ota_net.h               TCP 服务器接口
│   │   ├── update.h                固件搬运与跳转
│   │   ├── flags.h                 健康标志读写
│   │   ├── lcd.h / cfont.h          LCD 显示 + 字库
│   │   └── ...
│   └── Src/
│       ├── Bootloader.c            ★ 启动判定、环形缓冲、主循环
│       ├── protocol.c              ★ 帧同步状态机、CRC、命令处理
│       ├── flash_if.c              ★ 扇区擦除、字编程、自检
│       ├── update.c                ★ 固件搬运、向量表重定位、跳转
│       ├── flags.c                 健康标志
│       ├── ota_net.c               ★ TCP 服务器（accept/recv/err 回调）
│       ├── main.c                  初始化顺序
│       └── ...
├── LWIP/                       ← lwIP 移植层
│   ├── App/lwip.c                  MX_LWIP_Init / MX_LWIP_Process
│   └── Target/
│       ├── ethernetif.c            ★ 零拷贝收发、自定义内存池
│       └── lwipopts.h              lwIP 配置（NO_SYS=1，TCP_MSS=1426）
├── Drivers/                    ← STM32F4xx HAL + CMSIS + PHY 驱动(lan8742/，见"硬件")
├── Middlewares/                ← lwIP 2.1.2 源码
├── MDK-ARM/                    ← Keil MDK 工程（编译产物已 gitignore）
├── tools/                      ← 上位机工具（见 tools/README.md）
│   ├── ota_proto.py                共享协议层（CRC 表与 boot 逐值相同）
│   ├── ota_host.py                 无界面 CLI
│   └── host_qt/                    PySide6 图形界面
└── OTA_ETH.ioc                 ← STM32CubeMX 工程配置
```

★ = 本项目自研的核心代码

---

## 快速开始

### 硬件

- STM32F407ZGTx（1MB Flash）
- LAN8720A PHY，**RMII** 接口
  > 注：CubeMX 生成的 PHY 驱动目录叫 `Drivers/BSP/Components/lan8742/`，那是 ST 模板的默认命名
  > （取自 ST 自家评估板），**不代表实际芯片型号**。LAN8720A 与 LAN8742A 寄存器兼容，同一份驱动通用。
- 可选：HC-05 蓝牙模块接 USART3
- 可选：FSMC 并口 LCD 用于显示进度

### 编译

1. 用 **Keil MDK** 打开 `MDK-ARM/OTA_ETH.uvprojx`
2. 确认目标芯片包已安装（STM32F4xx_DFP）
3. Build → 生成 `MDK-ARM/SLOTA_ETH.bin`（可用 `fromelf` 后处理）

> **首次编译前请确认链接脚本 IROM1 Size = `0x20000`**（见 [Flash 分区](#flash-分区) 的警告）。

### 运行

上电后 Bootloader 先运行，并在 **5 秒窗口**内等待上位机连接：

- 收到合法 `OTA_START` → 进入下载会话，常驻等待升级
- 窗口内无人连接 → 跳转到 `APP_RUN_ADDR` 运行既有 APP

### 网络配置

boot 侧为 **TCP 服务器**，静态 IP 在 `LWIP/App/lwip.c` 的 `MX_LWIP_Init()` 中配置：

| 项 | 值 |
|---|---|
| 本机 IP | `192.168.1.2` |
| 子网掩码 | `255.255.255.0` |
| 监听端口 | `5001`（`ota_net.h` 的 `OTA_TCP_PORT`） |

上位机作为 **TCP client** 主动连接。

### 串口配置

USART3：**19200 8N1**（`Core/Src/usart.c`）。这个波特率是配合 **HC-05 蓝牙透传**的 ——
蓝牙 SPP 在弱链路下需降速换稳定性，代价是单帧传输时间变长（见 [已知限制](#已知限制)）。

---

## 上位机用法

依赖：**Python 3.x**；串口模式需 `pyserial`，图形界面需 `PySide6`。

```bash
py -m pip install pyserial PySide6
```

### 图形界面

```bash
py tools/host_qt/main.py
```

三通道选项卡 → 选择固件 `.bin` → 开始升级 → 日志 + 进度条。

### 无界面 CLI

```bash
# 以太网通道（推荐，最快）
py tools/ota_host.py --tcp 192.168.1.2:5001 firmware.bin

# 蓝牙通道（COM 口为 HC-05 配对后的 SPP 虚拟串口）
py tools/ota_host.py COM5 firmware.bin --baud 19200

# 功能性测试开关
py tools/ota_host.py --tcp 192.168.1.2:5001 fw.bin --probe       # 只探测通道是否通
py tools/ota_host.py --tcp 192.168.1.2:5001 fw.bin --inject-err  # 注入坏帧，验证重发
py tools/ota_host.py --tcp 192.168.1.2:5001 fw.bin --oversize    # 伪造越界固件，验证拒绝
py tools/ota_host.py --selftest                                  # CRC 一致性自检（不连设备）
```

> **固件上限 128KB**。超限时 `OTA_START` 会被拒绝。

详见 [`tools/README.md`](tools/README.md)。

---

## 关键设计点

### 1. 掉电保护：A/B 双分区 + 健康标志

升级流程中**任何时刻断电**，设备都不会变砖：

```
下载区 A 写入中 ──断电──► 复位 → 健康标志未置位 → 跳回 run 区旧固件 ✓
搬运到 run 区中 ──断电──► 复位 → 标志区状态不完整 → 跳回旧固件 ✓
新固件启动失败  ──────► 备份区 B 保留旧固件，可回退 ✓
```

关键在于**"新固件完整且校验通过"之前，运行区一直保持可用状态**，
半成品固件永远不会被当成有效固件启动。

### 2. 以太网零拷贝接收

STM32Cube 生成的 `ethernetif.c` 采用 **自定义 pbuf + 私有内存池**方案：

```c
typedef struct {
  struct pbuf_custom pbuf_custom;              /* pbuf 结构体 */
  uint8_t buff[(ETH_RX_BUF_SIZE + 31) & ~31];  /* DMA 缓冲区，紧跟在结构体之后 */
} RxBuff_t;

LWIP_MEMPOOL_DECLARE(RX_POOL, 12, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");
```

分配时正向取偏移，让 DMA 直接写入缓冲区：

```c
*buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
```

回收时反向找出 pbuf 基址（也就是整块内存的基址）：

```c
p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
```

**数据自始至终没有移动过，移动的只是指针。** 代价是生命周期耦合 ——
`pbuf` 不再由 lwIP 标准分配器管理，必须挂自定义释放回调
`pbuf_free_custom()` 归还到 `RX_POOL`。

> 释放用的是 `p`（块基址），**不是 `p->payload`**。pbuf 描述的是"其中的有效数据"，
> 但归还的是整块内存 —— 两者不是一回事。

### 3. 通道与协议解耦

`protocol.c` 不包含任何外设头文件：

- **收** —— 上层把字节流喂进 `Proto_Feed()`，不关心来自 UART 还是 TCP
- **发** —— 上层用 `Proto_SetTx()` 注入发送函数，协议层不直接碰外设

于是 UART 与 ETH **共用同一个状态机、同一套命令处理**，新增通道（如 WiFi 模组透传）
只需再提供一个 `Proto_Tx_t` 和一个喂数据的地方。

> **发送失败时协议层不重发** —— 重发由上位机的 stop-and-wait 超时机制负责，
> 两端职责不重叠。

### 4. 共享环形缓冲的并发保护

USART3 是**中断驱动**的，而 lwIP 在**主循环轮询**。两者都会写同一个接收环，
所以临界区是必需的：

```c
uint32_t primask = __get_PRIMASK();
__disable_irq();
/* 操作环形缓冲 */
__set_PRIMASK(primask);
```

`Bootloader_RingWrite()` 返回**连续可写的字节数**，让调用者处理"环回绕"的情况。

---

## 已知限制

本项目定位是**单机位、有人值守的现场升级工具**，以下限制是有意接受的取舍：

| 限制 | 说明 | 影响 |
|---|---|---|
| **单连接模型** | `ota_accept_cb` 中新连接会 `tcp_abort` 掉旧连接 | 多机位并发升级需扩展会话管理 |
| **无设备侧会话超时** | boot 一旦进入下载会话就常驻等待，不会主动放弃 | 断线后需复位设备；换来的是不会被中途踢回 APP |
| **无看门狗** | 有意不加 —— Flash 擦除期间总线 stall、CPU 冻结，喂狗逻辑难以兼顾 | 卡死需人工复位 |
| **上位机无自动重连** | CLI 断线后重试耗尽即退出，不重连 | 断线需重跑，但设备侧支持原偏移续传 |
| **蓝牙通道吞吐低** | HC-05 需降至 19200 换稳定性，单帧 1036B 约需 0.55s | 整包升级耗时远高于以太网 |
| **链路曾协商为 10M 半双工** | 调试环境实测现象，不影响功能 | 限制以太网吞吐 |

> **为什么这些可以接受**：失败模式是**"卡住"而非"变砖"**（见设计点 1），
> 而设备就在操作者手边，复位键触手可及。**协议的复杂度应该花在会变砖的风险上。**

---

## 技术栈

| 类别 | 选型 |
|---|---|
| **MCU** | STM32F407ZGTx（Cortex-M4，168MHz） |
| **PHY** | LAN8720A，RMII 接口 |
| **TCP/IP** | lwIP 2.1.2，**`NO_SYS=1`** 裸机模式（无 RTOS，主循环轮询） |
| **lwIP 配置** | `LWIP_NETCONN=0`、`LWIP_SOCKET=0`（仅用 raw/callback API）、`TCP_MSS=1426` |
| **无线通道** | HC-05 蓝牙 SPP 透传，UART 19200 8N1 |
| **Flash 操作** | 直接寄存器级 IAP（HAL_FLASH_Program 字编程） |
| **校验** | CRC16-MODBUS（poly 0xA001，init 0xFFFF，查表） |
| **显示** | FSMC 并口 LCD |
| **工具链** | Keil MDK（ARMCC）+ MicroLIB |
| **配置工具** | STM32CubeMX（`OTA_ETH.ioc`） |
| **上位机** | Python 3 + pyserial + PySide6 |

---

## 许可

本项目为个人学习/作品项目。其中 `Drivers/` 与 `Middlewares/` 下的
STM32 HAL、CMSIS、lwIP 源码版权归各自原作者所有（STMicroelectronics / Swedish
Institute of Computer Science），遵循其原始许可协议。
