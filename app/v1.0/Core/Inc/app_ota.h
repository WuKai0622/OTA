/* ============================================================================
 * app 侧的 OTA 触发口 —— 问题④里"App 那一半"
 *
 * 背景（一句话）：boot 只在上电后的 **5 秒窗口**里等上位机的 OTA_START，窗口一过
 * 它就跳进 app 常驻运行。于是"设备正跑着 app 的时候，上位机才点开始升级"这件事
 * 在 app 这一侧需要有人接住 —— 接住它的就是本文件。
 *
 * 做法：app 常驻轮询 SPI（ESP32-C3 就在那条链路的另一头），
 *       看到 C3 有字节要交给我们（MISO 头里的 RX_LEN > 0）
 *       ⇒ 说明 PC 正对着 C3 的 5001 端口说话 ⇒ NVIC_SystemReset() 回 boot。
 *       复位之后 boot 会重新打开 5 秒窗口、自己轮询 SPI，
 *       上位机因超时重发的那一帧正好落进新窗口里，升级照常走完。
 *
 * ★ app 侧**不解析帧**（连 0xA5 0x5A 都不看），理由写在 app_ota.c 里。
 *
 * ★★ 本文件与 app_ota.c 在 **v1.0 与 v1.1 里必须逐字节相同** —— 两个工程只差
 *    LCD 演示任务，与 OTA 触发毫无关系。改了其中一份就必须同步另一份：
 *    两边行为不一致会以"某一个 app 升不上去"的形式冒出来，
 *    而那正是最容易被当成硬件/接线问题去查的现象。
 * ==========================================================================*/
#ifndef __APP_OTA_H
#define __APP_OTA_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：把片选 PB12 配成推挽输出（空闲高），并清零事务缓冲。
 * ★ 必须在 MX_SPI2_Init() 之后调用（它要用 hspi2）。
 * ★ 为什么片选要在这里自己配：app 的 CubeMX 工程里根本没有这个脚 ——
 *   boot 的 CubeMX 把 PB12 标成了 SPI2_CS 并配好，app 的没有
 *   （app 的 gpio.c 一个脚都没配，fsmc.c 只碰 F/G/E/D 四个口）。
 *   详见 app_ota.c 里 APP_OTA_CS_* 那一段。 */
void AppOta_Init(void);

/* 轮询任务体，**永不返回**。一般不需要直接调它 —— 用下面的 CreateTask()。
 * 每 APP_OTA_POLL_MS 跑一笔 264B 的 SPI 事务；只要从 C3 收到一个字节就复位。 */
void vAppOtaTask(void *arg);

/* 创建轮询任务。在 vTaskStartScheduler() 之前调一次即可。
 * ★ 用的是**静态分配**（TCB 与栈都在 .bss，不占 FreeRTOS 堆），
 *   理由见 app_ota.c 里 AppOta_CreateTask() 的注释 —— 一句话：本工程的堆只有 4KB，
 *   走动态分配会把余量压到约 0.4KB，而堆不够时 xTaskCreate 只是**静默返回失败**。 */
void AppOta_CreateTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_OTA_H */
