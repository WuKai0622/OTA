# STM32F407 三通道 IAP 在线固件升级 Bootloader

基于 **STM32F407ZGTx + LAN8720A** 的 Bootloader，支持经 **以太网(TCP)**、**串口(USB-TTL 直连)** 与 **WiFi(ESP32-C3 SPI 桥)** 三条链路远程升级应用固件 —— **不拆机、可远程、掉电不变砖**。

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

- **不拆机** —— 通过设备已有的通信链路（网口 / 串口 / WiFi）升级
- **可远程** —— 网口支持 TCP 直连，USB-TTL 串口免网络就地升级，WiFi 支持完全无线
- **掉电不变砖** —— A/B 双分区 + 健康标志，任意时刻断电复位都能回到可用固件
- **弱链路可用** —— 逐帧确认 + 超时重传 + 断点续传，丢包不导致整包重来

**三通道的价值**：现场有网线就用网口（最快、免配对）；手边只有 USB-TTL 转接板就用串口（免网络、免配对）；有 WiFi 路由器就用 ESP32-C3 桥接（彻底无线，且 C3 只做**与内容无关的字节透传**，换任何上层协议都不用重烧它）。**三条链路共用同一套协议状态机**，上位机侧只需切换传输层。

---

## 功能特性

| 特性 | 说明 |
|---|---|
| **三通道统一协议** | 同一套帧协议状态机跑在 UART(USART3)、ETH(TCP) 与 SPI(ESP32-C3 桥) 上，与物理通道完全解耦 |
| **会话绑定选路** | 收到 `OTA_START` 时**锁存**发起通道（**仅在没有活动会话时**，避免被拒的 `START` 劫持在跑会话的 ACK 通道），此后所有 ACK 只走该通道 —— 避免"挂着闲置 ETH 连接却走 WiFi 升级"时 ACK 发错地方 |
| **升级令牌** | `OTA_START` 数据区尾部带 8 字节令牌，**在擦除任何扇区之前**校验 —— 拦掉端口扫描/拿错固件的上位机误触发（强度仅到"防误触发"，见[已知限制](#已知限制)） |
| **A/B 双分区 + 健康标志** | 新固件先落下载区，校验通过才搬入运行区；掉电可回退，不由半成品固件启动 |
| **逐帧 stop-and-wait** | 每帧独立 ACK，丢帧/坏帧只重发单帧，不必整包重传 |
| **幂等重传 / 断点续传** | 每帧携带起始偏移，已提交帧直接回 ACK 不重写 Flash；断线后可原偏移续传 |
| **下载会话空闲上限** | 常驻下载循环 30s 收不到任何字节即拆会话并跳回运行区 APP —— 设备不会永久卡在 `DL:` |
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

### 三通道汇流

```
                    ┌──────────────┐
   USB-TTL 串口 ────►│  USART3 中断 │──┐
   (115200 8N1)      └──────────────┘  │
                                       │
                    ┌──────────────┐  │   临界区保护
   以太网 TCP ──────►│ lwIP 主循环  │──┼──► 同一个 8KB 接收环形缓冲 ──► Proto_Feed()
                    │   轮询       │  │         （单一状态机）
                    └──────────────┘  │
                                       │
                    ┌──────────────┐  │
   ESP32-C3 ────────►│ SPI2 主循环  │──┘   ← F407 做主机，主动发时钟
   (WiFi ↔ TCP)      │   轮询       │         不轮询就完全收不到字节
                    └──────────────┘
```

> ⚠️ **ETH 中断在本工程中未使能** —— 以太网收包完全靠主循环轮询 `MX_LWIP_Process()`。
> 而 USART3 是中断驱动的，所以两者访问共享环形缓冲时必须用临界区保护
> （`__get_PRIMASK` / `__disable_irq` / `__set_PRIMASK`）。
>
> ⚠️ **SPI 通道必须被周期性调用**（`OTA_Spi_Poll()`），因为 SPI 的时钟由**主机 F407** 发出：
> 不主动发起事务，ESP32-C3 那边收到的字节就永远上不来。它没有中断能推给我们。
> 这是三条通道里唯一"**不主动问就没有数据**"的一条。

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
| `CMD_OTA_START` | `0x0001` | PC → boot | 固件总长 (u32) **+ 令牌 (8B)** | 目标区（0 = A） |
| `CMD_OTA_DATA` | `0x0002` | PC → boot | 固件块 (≤1024B) | **本帧起始偏移 (u32)** |
| `CMD_OTA_END` | `0x0003` | PC → boot | 固件总长 (u32) | — |
| `CMD_RSP_ACK` | `0x0100` | boot → PC | 状态码 (1B) | 同请求帧的 RSV |

状态码：`RSP_OK = 0` / `RSP_ERR = 1`

### 升级令牌

`OTA_START` 的数据区是 **12 字节**：固件总长 u32（4B）+ 令牌（8B）。
令牌在**擦除任何扇区之前**校验，不符则直接回 `RSP_ERR`、一个扇区都不动。

> 🔒 **令牌的具体值只在源码里** —— 本 README 与上位机的界面提示都不给出。要填的时候去
> [`Core/Inc/protocol.h`](Core/Inc/protocol.h) 取 `OTA_TOKEN_W0` / `OTA_TOKEN_W1`，
> 两个 u32 按小端拼成 8 字节。文档里写明值等于把"填令牌"这个动作代劳了，令牌也就白设了。

- boot 侧：`protocol.h` 的 `OTA_TOKEN_W0` / `OTA_TOKEN_W1`（两个 u32，小端拼成 8 字节）
- 上位机侧：`tools/ota_proto.py` 的 `OTA_TOKEN`，由同一对 u32 用 `struct.pack("<II", ...)`
  拼出，并断言展开后确实等于预期的 8 个字节 —— 避免手写 hex 被按大端阅读顺序写错
- **上位机里怎么填**：令牌是二进制（可能含 `0x00`），不是可读串，所以输入框/`--token` 收的是
  **16 位 hex**（带空格分隔亦可）；日志回显同样用 hex，不要指望它是可读文本
- **两端必须同版本**：老上位机只发 4 字节 DATA，会被新 boot 拒绝（fail-closed，正确方向）；
  新上位机对老 boot 则令牌被静默忽略

> ⚠️ 令牌的强度只到"**拦截误触发**"（端口扫描、拿错固件的上位机），**不是安全边界** ——
> 本仓库公开，常量明文可见。详见[已知限制](#已知限制)。

### 升级流程

```
① OTA_START(总长 L, 令牌, 区 A)
      boot: 校验数据区长度(12B) → 校验令牌 → 校验 L → 擦除整个下载区（典型 1s，最坏 4s）→ ACK(OK)
      ⚠️ 擦除期间 Flash 总线 stall、CPU 冻结，上位机超时必须 ≥ 擦除耗时（实测 1.03s、
         规格最坏 4s）。当前 START_TIMEOUT = 4.5s，且它**不是越大越好** —— 上界由 5s
         等待窗口给出（T ≤ D + 5s）。完整推导见 tools/README.md 的"两个必须知道的超时约束"。

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
bootloader/                     ← 本目录即 bootloader 工程根
├── Core/                       ← 应用核心（本项目的主要工作）
│   ├── Inc/
│   │   ├── Bootloader.h            启动流程、接收环、任务调度
│   │   ├── protocol.h              ★ 帧协议定义（与通道解耦）
│   │   ├── flash_if.h              ★ Flash 分区表 + 扇区操作
│   │   ├── ota_net.h               TCP 服务器接口（ETH 通道）
│   │   ├── ota_spi.h               ★ SPI 通道（F407 主机 ↔ ESP32-C3 从机）
│   │   ├── ota_chan.h              ★ 通道枚举 + 接收汇点类型（打破循环依赖）
│   │   ├── update.h                固件搬运与跳转
│   │   ├── flags.h                 健康标志读写
│   │   ├── lcd.h / cfont.h          LCD 显示 + 字库
│   │   └── ...
│   └── Src/
│       ├── Bootloader.c            ★ 启动判定、环形缓冲、三通道汇流、主循环
│       ├── protocol.c              ★ 帧同步状态机、CRC、命令处理、会话绑定
│       ├── flash_if.c              ★ 扇区擦除、字编程、自检
│       ├── update.c                ★ 固件搬运、向量表重定位、跳转、A/B 决策
│       ├── flags.c                 健康标志
│       ├── ota_net.c               ★ TCP 服务器（accept/recv/err 回调）
│       ├── ota_spi.c               ★ SPI 事务循环、发送队列、链路诊断
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
- 可选：USB-TTL 串口模块接 USART3（115200 8N1）
- 可选：FSMC 并口 LCD 用于显示进度

### 编译

1. 用 **Keil MDK** 打开 `MDK-ARM/OTA_ETH.uvprojx`
2. 确认目标芯片包已安装（STM32F4xx_DFP）
3. Build → 生成 `MDK-ARM/SLOTA_ETH.bin`（可用 `fromelf` 后处理）

> **首次编译前请确认链接脚本 IROM1 Size = `0x20000`**（见 [Flash 分区](#flash-分区) 的警告）。

### 运行

上电后 Bootloader 先运行，并在 **5 秒窗口**内等待上位机连接：

- 收到合法 `OTA_START`（长度 12B + 令牌相符）→ 进入下载会话，常驻等待升级
- 窗口内无人连接 → 跳转到 `APP_RUN_ADDR` 运行既有 APP
- 下载会话中**连续 30 秒没有任何字节到达**（`BOOT_DL_IDLE_MS`）→ 判定上位机已放弃
  （拔线 / 关窗口 / 上位机死机 / 换通道），拆除会话并尝试跳回运行区 APP，
  而不是永远停在 `DL:` 等一个不会来的数据帧
  > 一旦这样退回 APP，想再升级就需要**复位设备**（APP 自身没有网络能力，这是既有架构限制）。
  > 30 秒的取值依据见 `protocol.h` / `Bootloader.h` 的注释：正常会话里最长的**合法**静默
  > 约 6 秒（stop-and-wait 的重发前静默），30 秒有约 5 倍余量。

### 网络配置

boot 侧为 **TCP 服务器**，静态 IP 在 `LWIP/App/lwip.c` 的 `MX_LWIP_Init()` 中配置：

| 项 | 值 |
|---|---|
| 本机 IP | `192.168.1.2` |
| 子网掩码 | `255.255.255.0` |
| 监听端口 | `5001`（`ota_net.h` 的 `OTA_TCP_PORT`） |

上位机作为 **TCP client** 主动连接。

### 串口配置

USART3：**115200 8N1**（`Core/Src/usart.c`）。USB-TTL 直连 PC 时无需降速，PC 侧就是一个普通
COM 口。改波特率必须两侧同步改 —— boot 的 `usart.c`/`OTA_ETH.ioc` 与上位机的 `--baud` 或界面输入。

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

三通道选项卡 → 选择固件 `.bin` → **填写令牌**（必填，16 位 hex，取值见 `Core/Inc/protocol.h`）→ 开始升级 → 日志 + 进度条。

### 无界面 CLI

令牌是**必填项**，除 `--selftest` 外每条命令都要带 `--token <16位hex>`（漏填或凑不足 8 字节会在发第一个字节前报错退出）。取值见 `Core/Inc/protocol.h` 的 `OTA_TOKEN_W0/W1`。

```bash
# 以太网通道（推荐，最快）
py tools/ota_host.py --tcp 192.168.1.2:5001 firmware.bin --token <16位hex>

# 串口通道（USB-TTL 直连，PC 上是一个普通 COM 口；115200 与 CLI 默认值一致）
py tools/ota_host.py COM5 firmware.bin --token <16位hex>

# 功能性测试开关
py tools/ota_host.py --tcp 192.168.1.2:5001 fw.bin --token <16位hex> --probe       # 只探测通道是否通
py tools/ota_host.py --tcp 192.168.1.2:5001 fw.bin --token <16位hex> --inject-err  # 注入坏帧，验证重发
py tools/ota_host.py --tcp 192.168.1.2:5001 fw.bin --token <16位hex> --oversize    # 伪造越界固件，验证拒绝
py tools/ota_host.py --selftest                                                   # CRC 一致性自检（不连设备，不需要令牌）
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
| **下载会话 30s 空闲上限** | `BOOT_DL_IDLE_MS = 30000`：连续 30s 无字节到达即拆会话并跳回运行区 APP | 上位机中途停顿 >30s 会被判为放弃、退回 APP，**想再升级需复位设备**；换来的是设备不会永久卡在 `DL:`（长时间无人值守所必需） |
| **令牌只能拦误触发** | `OTA_TOKEN_W0/W1` 硬编码在公开源码里，且帧格式与 CRC 表也在 `tools/` 公开 | **不是安全边界**：挡不住有意的攻击者。暴露到不可信网络的部署必须先走 `#ifndef` 把令牌挪出仓库，并补传输层加密/认证 |
| **无看门狗** | 有意不加 —— Flash 擦除期间总线 stall、CPU 冻结，喂狗逻辑难以兼顾 | 卡死需人工复位 |
| **上位机无自动重连** | CLI 断线后重试耗尽即退出，不重连 | 断线需重跑，但设备侧支持原偏移续传 |
| **串口通道吞吐有限** | 115200 8N1 线速上限 11.25 KB/s；实测传输段 52.5 kbps（约 6.4 KB/s），28 帧共 4.37 s | 含擦除整包约 5.5 s，仍高于以太网 |
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
| **串口通道** | USB-TTL 直连，USART3 **115200 8N1** |
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
