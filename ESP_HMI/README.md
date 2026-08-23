# ESP_HMI

这是从只读参考工程 `E:\MotorControl\ESP32_LVGL\ESP32_LVGL` 独立建立的 ESP32 HMI 工程，保留了原有 BSP、LVGL、Wi-Fi 与 MQTT 功能。

电机通信分层如下：

```text
LVGL / MQTT
    -> CommMgr_ESP
        -> CAN_ESP
        -> USART_ESP
```

- `main/comm_mgr_esp.*`：唯一的物理链路选择点，状态为 `NONE / CAN / USART`；上层只能使用其统一控制与状态接口。
- `main/can_esp.*`：经典 CAN 500 kbit/s 的收发、协议编解码及 Bus-Off 处理。
- `main/usart_esp.*`：USART 115200 bit/s 的收发、协议编解码及 CRC16 检查。
- `main/motor_can_protocol.h` 与 `main/motor_uart_protocol.h`：与 STM32 工程中的副本保持完全一致。

运行中切换速度/位置模式时，ESP32 只发送统一模式命令；STM32 `MotorMgr` 负责完成减速、MOE 关闭、模式切换和重新启动的安全顺序。

`ZERO_POSITION` 仅为协议兼容保留；STM32 会拒绝该命令。需要回到零度时使用普通位置目标 `SET_POSITION=0`。
