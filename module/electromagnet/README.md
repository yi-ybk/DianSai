# 电磁铁 MOS 驱动模块

本目录提供天猛星 MSPM0G3507 对大功率 MOS 管驱动模块的基础开关控制。

## 1. 配置

| 项目 | 配置 |
| --- | --- |
| 控制引脚 | PB1 |
| GPIO 模式 | 推挽数字输出 |
| 有效电平 | 高电平有效 |
| 上电初始化 | 低电平，电磁铁关闭 |
| SysConfig 实例 | `GPIO_ELECTROMAGNET` |

PB1 已从天猛星扩展排针引出，并且在当前工程中未被其他外设占用。它不与
SWD、NRF24L01、HC-12、CAN、OLED、电机、编码器或灰度传感器的现有引脚
冲突。

## 2. 接线

| MOS 驱动模块 | 连接 |
| --- | --- |
| IO / PWM | 天猛星 PB1 |
| GND（信号地） | 天猛星 GND |
| DC+ | 外部 5 V 电源正极 |
| DC- | 外部 5 V 电源负极 |
| OUT+ | 电磁铁正极 |
| OUT- | 电磁铁负极 |

资料中的 P20/15 电磁铁额定电压为 5 V，因此驱动模块的 DC+/DC- 应使用
满足电磁铁电流要求的 5 V 电源。外部电源负极、驱动模块信号地和天猛星
GND 必须共地。

注意：

- 不要用 PB1 或其他 MCU GPIO 直接给电磁铁供电。
- 不要把 5 V 接入 PB1；PB1 只连接驱动模块的 IO/PWM 逻辑输入。
- 不要使用天猛星的 3.3 V 给驱动模块和电磁铁供电。
- 如果外部 5 V 电源的电流较大，建议在线路中加入保险丝和独立电源开关。
- MCU 复位期间 GPIO 会短暂处于高阻态。若实际模块在复位时出现误吸合，
  可在 PB1 与 GND 之间增加一个 10 kΩ 下拉电阻。

## 3. 初始化

`SYSCFG_DL_init()` 会完成 PB1 的 GPIO 配置，并在使能输出前写入低电平。
应用层随后调用 `ElectromagnetInit()`，再次确保输出保持关闭：

```c
#include "electromagnet.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();
    ElectromagnetInit();

    for (;;)
    {
        /* 应用逻辑 */
    }
}
```

## 4. 基本接口

```c
ElectromagnetOn();          /* 打开 */
ElectromagnetOff();         /* 关闭 */
ElectromagnetSet(true);     /* 打开 */
ElectromagnetSet(false);    /* 关闭 */
ElectromagnetToggle();      /* 翻转状态 */

if (ElectromagnetIsOn())
{
    /* 当前控制输出为高电平 */
}
```

这些接口只控制 GPIO，不包含阻塞延时，可以在普通主循环或 FreeRTOS 任务中
调用。多个任务可能同时控制电磁铁时，应由一个任务统一管理，或在上层增加
互斥和状态机，避免不同任务反复覆盖输出。

## 5. 安全建议

电磁铁属于感性负载。当前资料所示驱动板已经包含 MOS 管和相关保护器件，
仍应确认模块方向、外部电源极性以及 OUT+/OUT- 接线无误后再上电。首次测试
建议使用具有限流功能的 5 V 电源，并先设置较小的电流限制。
