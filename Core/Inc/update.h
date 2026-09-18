#ifndef __UPDATE_H
#define __UPDATE_H

#include <stdint.h>

/* OTA_END 验收通过后调用：把下载区 A 的新固件作为"候选"放入运行区试运行。
 * 流程：记录声明候选(seq/run_ver，同时自动清健康) → A→运行区搬运 → 向量检查 →
 * 通过则跳转(不返回)；失败则从 B 就地恢复后返回停留(boot 常驻可重发)。
 * 跳转成功本函数不返回。 */
void Upd_OnDownloadComplete(uint32_t total_len);

/* 上电等待窗口超时且无下载会话时调用：按标志区做启动决策
 *   - 有 B：run 是好版(版本同 B)→直接跳；候选健康→固化进 B 后跳；
 *           候选未确认/run 坏/版本倒退→拷 B→run 回退后跳；
 *   - 无 B：run 有有效固件→种子化 B(兼容旧版直接下载)后跳；
 *   - 都无：返回停留(boot 常驻等待下载/SWD)。
 * 跳转成功本函数不返回。 */
void Upd_StartupLaunch(void);

#endif /* __UPDATE_H */
