# 上位机与联调工具

对应帧协议方案 [OTA帧协议设计方案.md](../docs/OTA帧协议设计方案.md)。帧格式：`包头2 + 命令2 + 长度2 + 数据N + 保留4 + CRC16(2)`，CRC16-MODBUS 覆盖 `[命令..保留]`。

当前目标平台为 **F407ZGT6**（工程 `../OTA_ETH`）：boot 侧同时挂了 **UART(USART3)** 与 **ETH(lwip/TCP 服务器)** 两条通道，两条通道的字节流汇入同一个接收环，帧协议只有一份。

## 文件

| 文件 | 作用 |
|---|---|
| `ota_proto.py` | 共享协议层：CRC16-Modbus 查表 + 组帧/拆帧/分包。**CRC 表与 boot 的 `protocol.c` 逐值相同** |
| `ota_host.py` | 无界面 CLI：真机联调 boot 帧协议（支持 UART 与 TCP 两条通道，含坏帧/越界测试） |
| `host_qt/` | PySide6 图形上位机（UART / ETH / wifi 三通道） |

## 图形上位机（PySide6）

```bash
py -m pip install PySide6        # 一次性
py tools/host_qt/main.py         # 启动
```

界面：三个通道选项卡 → 选择固件 `.bin` → 开始升级 → 日志 + 进度条。

- **UART 串口**：直连 boot 的 USART3（有线直连 115200 8N1）。若这一头挂的是蓝牙透传模组，需降到 9600~19200。
- **ETH 网口**：boot 侧是 TCP 服务器（静态 IP `192.168.1.2`，端口 `5001`），上位机作为 client 主动连接。
- **wifi 模组**：同样是 TCP client，目标是模组被分配到的 IP。模组内的透传目标需自行配好。

## 无界面 CLI（boot 协议验证）

```bash
# UART 通道
py tools/ota_host.py COM5 MDK-ARM/SLOTA_ETH.bin        # 正常升级
py tools/ota_host.py COM5 x.bin --probe                # 只发 OTA_START 探测
py tools/ota_host.py COM5 x.bin --inject-err           # 破坏一个数据帧 -> 应触发重发
py tools/ota_host.py COM5 x.bin --oversize             # 伪造 >128KB 固件 -> START 应拒
py tools/ota_host.py COM5 x.bin --baud 9600            # 串口那头是蓝牙透传时降速

# TCP 通道（ETH / wifi）
py tools/ota_host.py --tcp 192.168.1.2:5001 x.bin
py tools/ota_host.py --tcp 192.168.1.50:5001 x.bin --probe

# 不连设备
py tools/ota_host.py --selftest                        # CRC 一致性自检
```

固件上限 **128KB**（`ota_proto.APP_MAX_SIZE`，须与 boot 的 `flash_if.h` 保持一致）。串口默认 115200 8N1。

## 流程

```
OTA_START(总长L, 目标区A) -> boot 擦除整个下载区（见下方超时说明） -> ACK(OK)
OTA_DATA(off, ≤1024B)     逐帧发，每帧等该偏移的 ACK(OK) 才发下一帧；超时/NACK 则重发本帧
OTA_END(总长L)            -> 累计字节==L -> ACK(OK) -> boot 搬运到运行区并跳转
```

boot 采用 **stop-and-wait 逐帧确认**：丢帧/坏帧由单帧重发兜住，不必整包重发。

## 两个必须知道的超时约束

- **`START_TIMEOUT = 8.0s`**（`ota_host.py` 与 `host_qt/ota_worker.py` 各一处）。F407 的最小擦除单位是**扇区**（最小 128KB），收到 OTA_START 后 boot 要先把整个下载区擦掉才会回 ACK —— 即使只传几十 KB 的固件也要擦掉一整块扇区，典型 1s、最坏 4s，且擦除期间 Flash 总线 stall、CPU 冻结。**不要改回 3s**：那会把正常擦除误判成无应答而重发，重发又触发一次全擦。
- **`OTA_DATA_CHUNK = 1024`** 对应 boot 侧 `TCP_MSS` 需为 `1460`（`../OTA_ETH/LWIP/Target/lwipopts.h`）。帧总长 1036B 加 TCP/IP 头约 40B 仍小于 MTU 1500，不会被 IP 分片；若 `TCP_MSS` 留在默认的 536，每个应用帧会被拆成 2 个 TCP 段。
