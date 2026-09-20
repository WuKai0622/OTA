# SPI↔TCP 桥（步骤③ 起就是这套，步骤④ 也是一样 —— 接真 OTA 时**只换 F407 那头**，
# C3 侧原样不动：它本来就是一条与内容无关的字节管道）。
#
# 步骤①② 的两套验证固件（tcp_echo.c/.h、spi_slave_loopback.c/.h）已删除 ——
# 那两步的结论已经沉淀成这里的 `spi_bridge.c` 与接线约定，留着只是死代码。
# 若要复现那两步的单项验证，按 `OTA/docs/OTA_WiFi_SPI通道方案.md` 第七节重建。
idf_component_register(SRCS "app_main.c"
                            "wifi_link.c"
                            "tcp_bridge.c"
                            "spi_bridge.c"
                    INCLUDE_DIRS ".")
