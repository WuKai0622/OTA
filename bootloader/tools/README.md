# 上位机与联调工具

帧格式：`包头2 + 命令2 + 长度2 + 数据N + 保留4 + CRC16(2)`，CRC16-MODBUS 覆盖 `[命令..保留]`。协议定义的权威来源是 [`../Core/Inc/protocol.h`](../Core/Inc/protocol.h)，项目总览见 [`../README.md`](../README.md)。

当前目标平台为 **F407ZGT6**（本仓库根目录即该工程）：boot 侧同时挂了 **UART(USART3)**、**ETH(lwip/TCP 服务器)** 与 **SPI(F407 作主，经 ESP32-C3 桥转 WiFi)** 三条通道，三条通道的字节流汇入同一个接收环，帧协议只有一份。

## 文件

| 文件 | 作用 |
|---|---|
| `ota_proto.py` | 共享协议层：CRC16-Modbus 查表 + 组帧/拆帧/分包。**CRC 表与 boot 的 `protocol.c` 逐值相同** |
| `ota_host.py` | 无界面 CLI：真机联调 boot 帧协议（支持 UART 与 TCP 两条通道，含坏帧/越界测试） |
| `host_qt/` | PySide6 图形上位机（UART / ETH / wifi 三通道） |
| `ota_timing_sim.py` | **离线时序验证（不接板子）**：用虚拟时钟跑真实的 `run_ota()`，扫"设备复位延迟"看 `START_TIMEOUT` 能覆盖多大范围、会不会多擦一遍 |

## 图形上位机（PySide6）

```bash
py -m pip install PySide6        # 一次性
py tools/host_qt/main.py         # 启动
```

界面：三个通道选项卡 → 选择固件 `.bin` → **填写令牌** → 开始升级 → 日志 + 进度条。

- **UART 串口**：直连 boot 的 USART3，**19200 8N1**（boot 固件里写死 19200，见 [`../Core/Src/usart.c`](../Core/Src/usart.c)）。⚠️ **`--baud` 的默认值是 115200，与 boot 不匹配**，走串口必须显式给 `--baud 19200`。这一头挂 HC-05 蓝牙透传时，SPP 虚拟串口的波特率设置不影响空中链路（模组串口侧另行配置），照给 `--baud 19200` 即可。
- **ETH 网口**：boot 侧是 TCP 服务器（静态 IP `192.168.1.2`，端口 `5001`），上位机作为 client 主动连接。
- **wifi 模组**：同样是 TCP client。ESP32-C3 默认 AP 模式（见 [`../../esp32/wifi/main/wifi_link.h`](../../esp32/wifi/main/wifi_link.h)）：自己开热点 `OTA_ESP32` / `12345678`，地址固定 **`192.168.4.1`**，端口 `5001`，先把 PC 连上该热点。改成 STA 模式则没有固定 IP，填模组打印出来的那个地址。
- **令牌**：必填，须与 boot 侧 [`../Core/Inc/protocol.h`](../Core/Inc/protocol.h) 的 `OTA_TOKEN_W0/W1` 一致。令牌是**二进制**（可能含 `0x00`）、不是可读串，所以按 **16 位 hex** 填（带空格分隔也可）。**取值只写在源码里，本文档不给** —— 写了就等于把"填令牌"代劳掉。**留空则拒绝开始**（不会预填默认值 —— 预填会让它退化成"看都没看就点开始"）。

## 无界面 CLI（boot 协议验证）

令牌是**必填项**，下面每条连设备的命令都要带 `--token <16位hex>`（唯一例外是 `--selftest`，它不连设备）。`<16位hex>` 填 [`../Core/Inc/protocol.h`](../Core/Inc/protocol.h) 里 `OTA_TOKEN_W0/W1` 小端拼出的那 8 字节。

> ⚠️ 下面的 `<16位hex>`、`x.bin`、`COM5` 是**占位符，不要照抄**。在 PowerShell 里 `<`
> 是保留的重定向运算符，把带尖括号的示例直接粘进终端会先报一个语法错。

```bash
# UART 通道（波特率必须显式给：boot 的 USART3 是 19200，CLI 默认 115200 不匹配）
py tools/ota_host.py COM5 x.bin --token <16位hex> --baud 19200              # 正常升级
py tools/ota_host.py COM5 x.bin --token <16位hex> --baud 19200 --probe      # 只发 OTA_START（见"已知问题"）
py tools/ota_host.py COM5 x.bin --token <16位hex> --baud 19200 --inject-err # 破坏一个数据帧 -> 应触发重发
py tools/ota_host.py COM5 x.bin --token <16位hex> --baud 19200 --oversize   # ⚠️ 当前不可用，见"已知问题"

# TCP 通道（ETH / wifi）
py tools/ota_host.py --tcp 192.168.1.2:5001 x.bin --token <16位hex>
py tools/ota_host.py --tcp 192.168.4.1:5001 x.bin --token <16位hex> --probe

# 不连设备（不需要令牌）
py tools/ota_host.py --selftest                                    # CRC 一致性自检
```

漏填 `--token`、或填的内容凑不出 8 字节（既不是 16 位 hex，长度也不是 8 个字符）时，CLI 在**发第一个字节之前**就报错退出（`令牌无效: ...`）—— 因为设备侧"令牌错"和"链路/固件有问题"都只表现为一个 `RSP_ERR`，不在本地拦掉就会在板子前反复排查错方向。注意这条校验只查**格式**，不查值对不对（上位机不知道 boot 里烧的是哪个令牌）。

固件上限 **128KB**（`ota_proto.APP_MAX_SIZE`，须与 boot 的 `flash_if.h` 保持一致）。串口通道的波特率必须与 boot 的 USART3 一致：**19200**（CLI 默认值 115200 不匹配，须显式 `--baud 19200`）。

### ⚠️ 当前版本的两个已知问题

- **`--probe` 不是无损探测** —— 它发的是**合法**的 START（长度与令牌都通过校验），而 boot 在回 ACK **之前**就擦掉了整个下载区（`protocol.c` 的 `Flash_EraseRegion`）。于是这一帧若落在 boot 的窗口里（例如刚上电就发），会擦掉下载区、建立会话，然后停在 `DL:` 等数据 —— 直到 30 秒空闲超时才自己跳回 APP。它只保证**不传固件数据**，不保证**不改设备状态**。
- **`--oversize` 当前不可用** —— 不带 `--probe` 时 `split_firmware()` 对超限固件直接 `raise ValueError`，**在发出第一个字节之前就崩**，"越界应被拒"这条测试跑不到；带 `--probe` 时又会被 `min(len(fw), APP_MAX_SIZE)` 夹成"恰好等于上限"，而 boot 判的是 `> APP_MAX_SIZE` → **会被接受**，结论正好相反。要验越界拒绝目前只能手工构造一个越界 START 帧。

## 离线时序验证（`ota_timing_sim.py`，不接板子）

`START_TIMEOUT` / `START_RETRY` 调多少合适，不用上板子试 —— 这个脚本用虚拟时钟跑**真实的 `run_ota()`**（只换时钟和链路对象，不复制一份逻辑，所以不会和真实实现漂移），扫一遍"设备复位延迟"给出结论：

```bash
py tools/ota_timing_sim.py                      # 扫 3/5/8 秒，看各自覆盖多大的复位延迟
py tools/ota_timing_sim.py --timeouts 4.5        # 单看某个候选值
py tools/ota_timing_sim.py --erase 4.0           # 按 ST 规格的最坏擦除时间，看会不会多擦一遍
py tools/ota_timing_sim.py --mode buffer         # 对照：APP 不吃 START（C3 的环留住它）
py tools/ota_timing_sim.py --verbose 2.0         # 打印"复位延迟 2s"那一次的真实升级日志
```

它量的是四件事：**哪些复位延迟连得上**、`START_RETRY` 有没有用（被测出基本没用）、**有没有多擦一遍**、以及 DATA 帧是否被擦除顶掉。模型有意简化了的部分都写在文件头的"不建模"里。

## 流程

```
OTA_START(总长L, 令牌8B, 目标区A) -> boot 校验令牌 -> 擦除整个下载区（见下方超时说明） -> ACK(OK)
OTA_DATA(off, ≤1024B)     逐帧发，每帧等该偏移的 ACK(OK) 才发下一帧；超时/NACK 则重发本帧
OTA_END(总长L)            -> 累计字节==L -> ACK(OK) -> boot 搬运到运行区并跳转
```

boot 采用 **stop-and-wait 逐帧确认**：丢帧/坏帧由单帧重发兜住，不必整包重发。

## 两个必须知道的超时约束

- **`START_TIMEOUT`**（`ota_host.py` 与 `host_qt/ota_worker.py` 各一处，**当前 4.5s**；原 8.0s，曾试 3.0s）。它同时管两件事：**① 重发节奏** —— 真正的判据是 `T ≥ D`，其中 D = "设备吃掉这一帧 → boot 窗口重新打开"的延迟（APP 轮询 → 复位 → boot 初始化）；**② 等 START 的 ACK 的耐心** —— 那个 ACK 要等 boot 把整个下载区擦完才回来（实测 1.03~1.20s、ST 规格最坏 4s），约束是 `T ≥ 擦除耗时`，否则重发会砸在正在擦除的 boot 上导致**再擦一遍**。上界由 `BOOT_WAIT_MS = 5s` 给出（`T ≤ D + 5s`）—— 所以 **T 不是越大越好**：超过约 5s 后覆盖区间会从 `[0, T]` 变成 `{0} ∪ [T-5, T]`，即"复位越快越连不上"，这正是原值 8.0 的症状。可用区间 `[max(D, 擦除耗时), D + 5s]`；现取 **4.5**（覆盖 D ≤ 4.5s，且 4.5 > 4s 永不重擦）。
	**这些判据跑 `py tools/ota_timing_sim.py` 就能自己复现**（不接板子）：它会打印每个 T 覆盖的 D 区间、有没有多擦一遍。`--erase 4.0` 复现最坏擦除下的重复擦除，`--mode buffer` 看 APP 不吃 START 的对照。
- **`OTA_DATA_CHUNK = 1024`** 对应帧总长 1036B，boot 侧 `TCP_MSS` 必须 **≥ 1036** 才能保证一帧装进单个 TCP 段。当前 [`../LWIP/Target/lwipopts.h`](../LWIP/Target/lwipopts.h) 中为 **`1426`**；若留在默认的 536，每个应用帧会被拆成 2 个 TCP 段。
