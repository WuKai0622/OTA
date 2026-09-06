# 上位机与联调工具

对应帧协议方案 [OTA帧协议设计方案.md](../docs/OTA帧协议设计方案.md)。帧格式：`包头2 + 命令2 + 长度2 + 数据N + 保留4 + CRC16(2)`，CRC16-MODBUS 覆盖 `[命令..保留]`。

## 文件

| 文件 | 作用 |
|---|---|
| `ota_proto.py` | 共享协议层：CRC16-Modbus 查表 + 组帧/拆帧/分包。**CRC 表与 `Bootloader/protocol.c` 逐值相同** |
| `ota_host.py` | 无界面 CLI：真机联调 boot 帧协议（含坏帧/越界测试） |
| `host_qt/` | PySide6 图形上位机（UART / ETH / wifi 三通道） |

## 图形上位机（PySide6）

```bash
py -m pip install PySide6        # 一次性
py tools/host_qt/main.py         # 启动
```

界面：三个通道选项卡 → 选择固件 `.bin` → 开始升级 → 日志 + 进度条。

- **UART 串口**：直连 boot 的 USART1（115200 8N1），当前唯一可联调通道。
- **ETH / wifi 模组**：传输层已就绪（TCP client，复用同一帧协议）；等 boot 侧 ETH(lwip)/wifi(SPI) 驱动接入后即可使用。

## 无界面 CLI（boot 协议验证）

```bash
py tools/ota_host.py COM5 MDK-ARM/SLOTA.bin         # 正常升级
py tools/ota_host.py COM5 x.bin --selftest           # CRC 一致性自检（不连串口）
py tools/ota_host.py COM5 x.bin --inject-err          # 破坏一个数据帧 -> 应 NACK
py tools/ota_host.py COM5 x.bin --oversize            # 伪造 >15KB 固件 -> START 应拒
py tools/ota_host.py COM5 x.bin --probe               # 只发 OTA_START 探测
```

波特率固定 115200 8N1。

## 流程

```
OTA_START(总长L, A区) -> [全擦下载区] -> ACK(OK)
OTA_DATA(off, ≤1024B) ... 逐帧顺序发（帧间 ~60ms 延时给写 Flash 留窗）
OTA_END(总长L)        -> 累计字节==L -> ACK(OK)，否则 ACK(ERR) -> 整包重发
```

注意：boot 无逐帧 ACK。丢帧/坏帧表现为 OTA_END 的 ACK(ERR)，需整包重发。
