# SSD1306 OLED 模块使用说明

## 1. 模块功能

`oled_driver` 用于驱动 0.96 寸、4 针 I2C 接口的 SSD1306 OLED，支持硬件 I2C 和 GPIO 模拟软件 IIC。

当前功能包括：

- SSD1306 初始化、关闭和反色显示；
- 128 x 64 单色显存；
- 清屏、填充、像素和实心矩形绘制；
- 5 x 7 ASCII 字符与字符串显示；
- 有符号整数和浮点数显示；
- 180 度画面旋转。

当前字库只支持 ASCII 字符 `' '` 到 `'~'`，不支持中文、全角字符和自定义字体。

## 2. 依赖与工程配置

需要包含以下源文件：

```text
module/oled/oled_driver.c
module/oled/oled_driver.h
bsp/iic/bsp_iic.c
bsp/iic/bsp_iic.h
```

头文件搜索路径至少需要：

```text
-Imodule/oled
-Ibsp/iic
```

使用 EIDE 时，确认 `module/oled/oled_driver.c` 已加入项目源文件列表；仅添加头文件搜索路径不会参与编译。

仅使用软件 IIC 时可以关闭 `HAL_I2C_MODULE_ENABLED`，OLED 和 `bsp_iic` 仍可正常编译。使用硬件 I2C 时必须在 `stm32f4xx_hal_conf.h` 中启用该宏，并将对应 HAL I2C 源文件加入工程。

## 3. 硬件连接

常见 4 针 OLED 模块的连接如下：

| OLED 引脚 | STM32 连接 |
| --- | --- |
| `VCC` | `3.3V` |
| `GND` | `GND` |
| `SCL` | I2C SCL 或软件 IIC SCL |
| `SDA` | I2C SDA 或软件 IIC SDA |

模块默认使用 7 位 I2C 地址 `0x3C`。有些模块通过硬件配置可使用 `0x3D`，此时将 `dev_address` 设为 `0x3D`。

地址必须填写 7 位地址，不需要左移。例如填写 `0x3C`，不要填写 `0x78`。

## 4. 硬件 I2C 使用

先在 CubeMX 中配置对应的 I2C 外设和引脚，然后使用 HAL 的 I2C 句柄初始化 OLED。

硬件模式要求已经启用 `HAL_I2C_MODULE_ENABLED`；宏关闭时，`IICRegister()` 会拒绝硬件 I2C 配置并返回 `NULL`。

```c
#include "oled_driver.h"
#include "i2c.h"

static Oled_t oled = { OLED_OBJECT_DEFAULT };

static const OledInitConfig_t oled_config = {
    .i2c_handle   = &hi2c1,
    .iic_bus_mode = IIC_BUS_HARDWARE,
    .dev_address  = OLED_SSD1306_DEFAULT_ADDR,
    .width        = OLED_SSD1306_WIDTH,
    .height       = OLED_SSD1306_HEIGHT,
    .rotate_180   = false,
    .inverted     = false,
};

bool AppOledInit(void)
{
    return oled.init(&oled, &oled_config);
}
```

当 `dev_address`、`width` 或 `height` 填写为 `0` 时，模块分别使用默认值 `0x3C`、`128` 和 `64`。

硬件 I2C 的 OLED 传输使用 `IIC_BLOCK_MODE`，调用刷新函数会阻塞直到数据发送完成或底层超时。

## 5. 软件 IIC 使用

软件 IIC 不依赖 CubeMX 的 I2C 外设。`bsp_iic` 会自动使能 GPIO 时钟，并将 SCL、SDA 配置为开漏输出、内部上拉和高速模式。

```c
#include "oled_driver.h"

static Oled_t oled = { OLED_OBJECT_DEFAULT };

static const OledInitConfig_t oled_config = {
    .i2c_handle   = NULL,
    .iic_bus_mode = IIC_BUS_SOFTWARE,
    .soft_iic = {
        .scl = {
            .GPIOx    = GPIOB,
            .GPIO_Pin = GPIO_PIN_6,
        },
        .sda = {
            .GPIOx    = GPIOB,
            .GPIO_Pin = GPIO_PIN_7,
        },
        .delay_us = 2U,
    },
    .dev_address = OLED_SSD1306_DEFAULT_ADDR,
    .rotate_180  = false,
    .inverted    = false,
};

bool AppOledInit(void)
{
    return OledInit(&oled, &oled_config);
}
```

`delay_us` 是软件 IIC 的半周期延时，填 `0U` 时默认使用 `2U`。它基于空循环，不是精确微秒计时；应根据系统主频、线长和显示稳定性调整。软件 IIC 仅支持阻塞模式和 `IIC_SEQ_RELEASE`，不支持中断、DMA 或保持总线的传输方式。

I2C 总线必须有上拉。模块内部上拉适用于低速、短连线场景；线长较长、干扰较大或提高速度时，应使用合适的外部上拉电阻。

## 6. 初始化与对象创建

推荐使用对象默认方法表：

```c
static Oled_t oled = { OLED_OBJECT_DEFAULT };

if (!oled.init(&oled, &oled_config))
{
    /* 初始化参数无效或 IIC 实例注册失败 */
}
```

也可以直接调用函数：

```c
static Oled_t oled = {0};

if (!OledInit(&oled, &oled_config))
{
    /* 初始化失败 */
}
```

不能对 `Oled_t oled = {0};` 直接调用 `oled.init()`，因为 `init` 函数指针为 `NULL`。

初始化会执行 SSD1306 命令序列、清空显存、刷新黑屏并打开显示。应在普通任务或初始化流程中调用，不要在中断服务程序中初始化或刷新 OLED。

## 7. 文本与数值显示

```c
oled.show_string(&oled, 0U, 0U, "HELLO", OLED_COLOR_WHITE);
oled.show_int(&oled, 0U, 2U, -12345, OLED_COLOR_WHITE);
oled.show_float(&oled, 0U, 4U, -3.14159f, 3U, OLED_COLOR_WHITE);
```

`OledShowString()`、`OledShowInt()` 和 `OledShowFloat()` 都会在写入显存后自动调用 `OledRefresh()`，只发送发生变化的显存页。对应的 `draw_string()`、`draw_int()`、`draw_float()` 只修改显存，适合连续显示多个字段后统一调用一次 `refresh()`。

字体宽度为 6 像素，包含 5 列字形和 1 列间隔；字体高度为 8 像素。因此在默认 128 x 64 屏幕上，一行最多显示 21 个 ASCII 字符，最多显示 8 行。

字符串超出右边界时会自动换行，超出底部时停止绘制。整数支持正数和负数；浮点数支持正数、负数和最多 6 位小数。浮点数为 NaN 时显示 `nan`，超过可格式化范围时显示 `ovf`。

显示函数不会清除旧字符。例如先显示 `1000` 再显示 `99`，末尾可能残留旧字符。覆盖数字时可先清除对应区域，或清屏后重新绘制整个界面。

批量刷新数值时可直接使用 `draw_int()` 和 `draw_float()`；以下是可直接使用的批量文本示例：

```c
oled.clear(&oled);
oled.draw_string(&oled, 0U, 0U, "System Ready", OLED_COLOR_WHITE);
oled.draw_string(&oled, 0U, 2U, "Mode: Auto", OLED_COLOR_WHITE);
oled.draw_int(&oled, 0U, 4U, -12345, OLED_COLOR_WHITE);
oled.draw_float(&oled, 10U, 4U, -3.14159f, 3U, OLED_COLOR_WHITE);
oled.refresh(&oled);
```

## 8. 显存绘制与刷新

`clear()` 会清空显存并自动刷新屏幕。以下函数只修改 RAM 显存，不会立即显示到 OLED：

```c
oled.fill(&oled, OLED_COLOR_WHITE);
oled.draw_pixel(&oled, 10U, 10U, OLED_COLOR_WHITE);
oled.draw_rect(&oled, 20U, 20U, 30U, 10U, OLED_COLOR_WHITE);
oled.draw_char(&oled, 0U, 0U, 'A', OLED_COLOR_WHITE);
oled.draw_string(&oled, 0U, 1U, "ASCII", OLED_COLOR_WHITE);
```

绘制完成后统一刷新：

```c
(void)oled.refresh(&oled);
```

`refresh()` 只发送自上次成功刷新后发生变化的显存页；每页高度为 8 像素。没有显存变化时不会产生 IIC 数据传输。需要在屏幕重新连接、显示内容可能失步等情况下强制发送完整显存时，调用 `refresh_all()`。不要绕过绘图接口直接修改 `oled.buffer[]`；直接修改不会自动标记脏页，确需直接写显存时应在写入后调用 `refresh_all()`。

当前底层 IIC 接口不向 OLED 模块返回设备应答或总线错误，因此即使屏幕未连接，刷新返回 `true` 也不能证明数据已成功到达 OLED。

坐标原点位于左上角，`x` 向右增加，`y` 向下增加。像素/图形接口和文本接口使用不同的坐标单位：

| 接口 | 坐标单位 | 通用范围 | 默认 128×64 屏幕范围 |
| --- | --- | --- | --- |
| `draw_pixel()`、`draw_rect()` | 像素 | `0 <= x < width`，`0 <= y < height` | `x=0..127`，`y=0..63` |
| `draw_char()`、文本和数值接口 | 6×8 字符网格 | `0 <= x < width/6`，`0 <= y < height/8` | `x=0..20`，`y=0..7` |

`width` 和 `height` 是初始化后的 `oled.data.width`、`oled.data.height`。例如 `show_string(&oled, 3U, 2U, ...)` 会从第4个字符列、第3个文本行开始显示，内部像素起点为 `(18, 16)`。字符串到达右边界后自动换到下一文本行，到达底部后停止绘制。像素接口的越界部分会被裁剪，`draw_rect()` 绘制的是实心矩形而不是空心边框。

颜色参数：

| 颜色 | 作用 |
| --- | --- |
| `OLED_COLOR_BLACK` | 清除像素 |
| `OLED_COLOR_WHITE` | 点亮像素 |
| `OLED_COLOR_INVERT` | 翻转目标像素 |

## 9. 显示控制

```c
oled.set_display(&oled, false);  /* 关闭显示，显存内容保留 */
oled.set_display(&oled, true);   /* 恢复显示 */

oled.set_inverted(&oled, true);  /* 全屏反色 */
oled.set_inverted(&oled, false); /* 恢复正常显示 */
```

`rotate_180` 仅在首次初始化时生效，当前模块没有运行时旋转接口。应在调用 `OledInit()` 前确定该配置。

## 10. 运行数据

```c
OledData_t data;

oled.get_data(&oled, &data);
```

`OledData_t` 包含有效显示宽高、最近一次字符串绘制后的字符列号和文本行号，以及显示开关状态。

## 11. 并发与中断限制

OLED 对象和 IIC 总线没有内部互斥保护。多个任务共享同一 OLED 或同一 IIC 总线时，应由应用层使用互斥锁，或约定只由一个显示任务访问。

不要在 EXTI、定时器或其他中断回调中调用 `OledRefresh()`、`OledShowString()`、`OledShowInt()` 或 `OledShowFloat()`。这些函数会进行阻塞 I2C 传输，软件 IIC 还会执行忙等延时。

推荐由周期显示任务集中刷新界面，例如每 50 ms 到 200 ms 刷新一次。脏页机制会跳过未变化页面，但过于频繁地刷新仍会占用 I2C 总线和 CPU 时间；`clear()` 和 `refresh_all()` 会强制传输完整显存。

## 12. 常见问题

### 初始化失败

检查以下项目：

- 硬件 I2C 模式下，`i2c_handle` 不能为 `NULL`；
- 软件 IIC 模式下，SCL 和 SDA 的端口与引脚必须有效；
- `iic_bus_mode` 必须是 `IIC_BUS_HARDWARE` 或 `IIC_BUS_SOFTWARE`；
- 宽度不能超过 128，高度不能超过 64，且高度必须是 8 的整数倍；
- OLED 地址是否为正确的 7 位地址。

### 屏幕不亮或没有内容

检查供电、电平、SCL/SDA 连线和 I2C 地址；确认已经调用初始化；确认 `draw_*` 后调用了 `refresh()`；确认显示颜色使用 `OLED_COLOR_WHITE`；确认项目实际编译进了 `oled_driver.c`。

### 显示方向或颜色不正确

初始化时设置 `.rotate_180 = true` 可以旋转画面，设置 `.inverted = true` 可以默认反色；运行中可调用 `set_inverted()` 切换反色。

### 文字乱码

当前仅支持 ASCII 5 x 7 字库。中文 UTF-8 字符、GBK 字符或其他非 ASCII 字符不会按预期显示。
