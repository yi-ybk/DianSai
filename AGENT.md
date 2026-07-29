## 总原则

本项目运行在以下环境：

- VS Code

### 环境确认命令

允许使用以下低风险只读命令确认环境：

```bash
pwd
whoami
hostname
git rev-parse --show-toplevel
git branch --show-current
git status --short
test -f /.dockerenv && echo "inside-docker" || true
cat /proc/1/cgroup 2>/dev/null | head -n 5
printf 'SSH_CONNECTION=%s\n' "${SSH_CONNECTION:+set}"
printf 'WSL_DISTRO_NAME=%s\n' "${WSL_DISTRO_NAME:+set}"
printf 'REMOTE_CONTAINERS=%s\n' "${REMOTE_CONTAINERS:+set}"
printf 'CODESPACES=%s\n' "${CODESPACES:+set}"
uname -s
```

默认压缩输出：

```text
Env: target=<local|container|remote-ssh|wsl|unknown> cwd=<path> git=<git-root> branch=<branch> path=<high|medium|low>
Target: modify=<local|container|remote|wsl> run=<local|container|remote|wsl> path=<workspace-path>
```

只有在以下情况才展开完整环境信息：

- `path=medium` 或 `path=low`
- 文件找不到
- 用户要求详细审计

展开格式：

```text
Environment check:
- Runtime target:
- Current working directory:
- Git root:
- Git branch:
- Container detected:
- Hostname:
- User:
- Path confidence:

Execution target:
- Modify files in:
- Run commands in:
- Workspace path visible to Codex:
- User-provided path, if any:
- Path mapping known:
```

`Path confidence` 判定标准：

- `high`: 当前 `cwd` 在目标 Git root 内；目标文件存在；只有一个候选路径。
- `medium`: Git root 可确认，但用户路径相对 `cwd` 不明确，或存在 VS Code multi-root / workspaceFolder 可能。
- `low`: Git root 不存在或失败；目标文件不可见

如果 `Path confidence` 不是 `high`，不要修改文件，先询问用户。

### 修改前强制停止条件

- 用户指定的文件在当前环境不可见。
- 文件路径存在多个候选位置。

### VS Code 工作区规则

- 不要假设 VS Code 打开的路径等于 Codex 当前工作路径。
- 必须用 `pwd` 和 `git rev-parse --show-toplevel` 确认 Codex 实际工作路径。

确认后，后续所有文件操作使用当前 Codex 可见路径

### 项目介绍
- 本项目为嵌入式项目，芯片为mspm0g3507
- 编译可调用任务D:\STM32Hal\DianSai_MSPM0G3507\.vscode\tasks.json

### 芯片适配与头文件规则

- 本工程唯一目标芯片为 MSPM0G3507，不得引入 STM32 HAL、STM32 CMSIS Device 或 STM32 CubeMX 生成文件。
- 新增外设驱动优先使用 TI DriverLib 原生接口，并直接包含：

```c
#include <ti/driverlib/driverlib.h>
```

- 使用 SysConfig 生成的外设实例、引脚和中断宏时，直接包含：

```c
#include "ti_msp_dl_config.h"
```

- 仅当现有 BSP 或模块需要继续使用 `HAL_*`、`GPIO_TypeDef`、`TIM_HandleTypeDef` 等 STM32 HAL 命名习惯时，才允许直接包含：

```c
#include "mspm0_hal_compat.h"
```

- `mspm0_hal_compat.h` 只负责 MSPM0 类型和接口到现有 HAL 命名习惯的兼容，不负责伪装芯片型号，也不代替 SysConfig 外设配置。
- 禁止创建或包含 `stm32f4xx_hal.h`、`stm32f407xx.h` 等伪 STM32 头文件，再由这些文件转包含 MSPM0 兼容层。
- 禁止为兼容旧工程而创建只转包含 `mspm0_hal_compat.h` 的 `main.h`、`gpio.h`、`tim.h` 等无实际声明的转发头文件。
- 纯算法代码不得依赖 `mspm0_hal_compat.h`、TI DriverLib 或 SysConfig 生成头文件；只包含自身实际使用的 C 标准库、CMSIS-DSP、FreeRTOS 等接口。
- 每个头文件和源文件必须直接包含自身所需声明对应的头文件，不得依赖其他头文件的传递包含。例如使用 `NULL` 时包含 `<stddef.h>`，使用定宽整数时包含 `<stdint.h>`。
- 修改现有兼容代码时可以保留公共 API 的 HAL 命名以避免破坏上层模块，但新增底层实现必须调用 MSPM0/TI 接口，不得加入 STM32 寄存器、外设句柄或芯片相关定义。
- 无法由通用兼容层正确表达的功能，如 DMA 通道、中断入口、定时器捕获比较或编码器模式，必须通过 SysConfig 和对应 MSPM0 BSP 明确实现，不得用固定成功返回值或空函数掩盖未实现状态。

### 要求
除非用户要求，不要修改代码文件中的注释，除非原有注释与现有代码的语义不符，则重新按照工程原有的注释风格修改、添加、删除注释
新增、修改代码或者模块时，要参考工程中原有的注释风格修改、添加、删除注释
