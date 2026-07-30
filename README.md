# DianSai_MSPM0G3507

基于 TI MSPM0G3507、FreeRTOS 和 TI DriverLib 的电赛机器人控制工程。工程面向嘉立创天猛星 MSPM0G3507 开发板，支持使用 CCS 编译、SysConfig 图形化配置外设，并使用 SEGGER J-Link 通过 SWD 下载程序。

## 1. 开发环境

建议使用与原工程一致的版本，避免 SysConfig 重新生成或编译器版本差异造成问题。

| 软件 | 当前工程版本/要求 |
| --- | --- |
| 操作系统 | Windows 10/11 64 位 |
| Code Composer Studio | CCS 21.0，工程使用 Theia 版 CCS |
| MSPM0 SDK | `2.11.00.07` |
| SysConfig | `1.28.0` |
| TI Arm Clang | `5.1.1.LTS` |
| VS Code | 可选，用于代码编辑、任务编译和 J-Link 烧录 |
| VS Code C/C++ 扩展 | 可选，用于头文件搜索和代码跳转 |
| SEGGER J-Link Software | 使用 VS Code 烧录时需要；当前已验证 J-Link Commander V8.50 |

> 工程配置文件明确引用 `mspm0_sdk@2.11.00.07`。安装其他 SDK 版本时，SysConfig 可能提示产品缺失或产生不同的生成代码，不建议直接忽略。

## 2. 分发包必须包含的内容

主工程依赖一个独立的 FreeRTOS 引用工程。发送工程时，不能只发送 `DianSai_MSPM0G3507` 文件夹，建议保持以下目录结构：

```text
DianSai_workspace/
├─ DianSai_MSPM0G3507/
└─ freertos_builds_LP_MSPM0G3507_release_ticlang/
```

两个文件夹必须位于同一个父目录，且名称不要修改：

- 主工程名：`DianSai_MSPM0G3507`
- FreeRTOS 引用工程名：`freertos_builds_LP_MSPM0G3507_release_ticlang`

不要分发以下本机生成内容：

- `Debug/`、`Release/`
- `.vscode/ipch/`
- `.theia/`、`.codex/`
- `*.out`、`*.map`、`*_linkInfo.xml`

这些内容已经由 `.gitignore` 排除。接收方应在自己的电脑上重新生成构建产物。

## 3. 安装 TI 开发环境

1. 安装 Code Composer Studio 21.0。
2. 安装 MSPM0 SDK `2.11.00.07`。
3. 确认 CCS 能识别以下产品：
   - MSPM0 SDK `2.11.00.07`
   - SysConfig `1.28.0`
   - TI Arm Clang `5.1.1.LTS`
4. 如果 CCS 没有自动识别 SDK，在 CCS 的产品发现设置中添加 SDK 安装目录，然后重启 CCS。

原开发机的 SDK 安装位置为：

```text
C:\TI\mspm0_sdk_2_11_00_07
```

接收方可以安装到其他目录，但必须确保 CCS 中的变量 `COM_TI_MSPM0_SDK_INSTALL_DIR` 指向实际 SDK 根目录。

## 4. 使用 CCS 导入和编译

### 4.1 导入工程

1. 打开 CCS，选择一个新的工作区。
2. 使用 `File -> Import Projects` 导入 `freertos_builds_LP_MSPM0G3507_release_ticlang`。
3. 再导入 `DianSai_MSPM0G3507`。
4. 确认 Project Explorer 中同时存在这两个工程。
5. 主工程的活动构建配置选择 `Debug`。

主工程通过 `.project` 和 `.cproject` 引用 FreeRTOS 工程。如果 CCS 报告：

```text
Referenced project 'freertos_builds_LP_MSPM0G3507_release_ticlang' does not exist in the workspace
```

说明 FreeRTOS 引用工程尚未导入、工程名被修改，或者两个工程没有被放入同一个交接包中。

### 4.2 SysConfig 外设配置

在 CCS 中双击：

```text
DianSai_MSPM0G3507.syscfg
```

即可使用图形界面修改 GPIO、PWM、UART、DMA、时钟和中断等配置。保存后，CCS 会在构建目录中生成：

```text
Debug/syscfg/ti_msp_dl_config.c
Debug/syscfg/ti_msp_dl_config.h
```

注意：

- 不要手工修改 `Debug/syscfg` 下的生成文件。
- 不要把 `Debug/syscfg` 当作需要交接的源代码。
- 外设配置应修改 `DianSai_MSPM0G3507.syscfg`，然后重新构建。

### 4.3 编译

在主工程上执行 `Build Project`，或者先执行 `Clean Project` 再执行 `Build Project`。成功后应生成：

```text
DianSai_MSPM0G3507/Debug/DianSai_MSPM0G3507.out
```

这个文件是后续调试和烧录使用的 ELF 文件。

## 5. 使用 VS Code 开发

VS Code 不是独立编译环境，当前任务仍然调用已安装的 CCS 命令行工具。

### 5.1 修改接收方电脑上的本机路径

当前仓库中有三处与原开发机有关的绝对路径，换电脑后需要检查。

#### `.vscode/ccs-build.ps1`

本工程默认使用以下 CCS 命令行工具：

```powershell
C:\TI\ccs2100\ccs\eclipse\ccs-server-cli.bat
```

换电脑后可在任务或脚本参数中传入新的 `-CcsCli` 路径。

CLI 专用工作区默认位于工程同级目录：

```powershell
D:\DianSai\m0\m0\.ccs-cli-workspace-compile-check
```

`$ws` 可以改成任意可写的本地目录，但不要指向工程源码目录，也不建议与正在运行的 CCS GUI 共用同一个工作区。

#### `.vscode/c_cpp_properties.json`

按实际安装位置修改：

- `compilerPath`：接收方的 `tiarmclang.exe`
- 所有 `C:/TI/mspm0_sdk_2_11_00_07/...`：接收方的 MSPM0 SDK 路径
- `D:/STM32Hal/freertos_builds_LP_MSPM0G3507_release_ticlang`：接收方的 FreeRTOS 引用工程路径

这部分只影响 VS Code 头文件搜索和代码跳转；真正的 CCS 编译路径由 CCS 产品配置决定。

### 5.2 VS Code 任务

在 VS Code 中执行 `Terminal -> Run Task`，可使用：

| 任务 | 作用 |
| --- | --- |
| `CCS: Build Debug` | 增量编译 Debug 配置 |
| `CCS: Full Build Debug` | 完整重新编译 Debug 配置 |
| `MSPM0: 一键编译并烧录` | 完整编译成功后自动烧录、校验并复位 |

默认构建快捷键 `Ctrl+Shift+B` 对应 `MSPM0: 一键编译并烧录`。

首次执行 VS Code 编译任务时，脚本会尝试把主工程和同级 FreeRTOS 引用工程自动导入 CLI 工作区。

## 6. 使用 J-Link 烧录

### 6.1 硬件连接

使用 SWD 连接开发板和 J-Link，至少需要：

- J-Link `VTref` 接目标板 `3.3 V`
- J-Link `GND` 接目标板 `GND`
- J-Link `SWDIO` 接 MSPM0G3507 `PA19`
- J-Link `SWCLK` 接 MSPM0G3507 `PA20`
- 建议同时连接 `nRESET`，便于异常程序运行时使用复位下连接

烧录前确认目标板已正常供电、J-Link 能检测到约 `3.3 V` 的 VTref，并确保 CCS、J-Link Commander 等程序没有同时占用同一个调试器。

### 6.2 从 VS Code 烧录

1. 安装 SEGGER J-Link Software。
2. 先确认工程能生成 `Debug/DianSai_MSPM0G3507.out`。
3. 按 `Ctrl+Shift+B`，或运行任务 `MSPM0: 一键编译并烧录`。

烧录脚本会自动从 Windows 注册表查找 `JLink.exe`。如果没有找到，可以在 PowerShell 中手动指定：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode/jlink-flash.ps1 -JLinkExe "C:\Program Files\SEGGER\JLink\JLink.exe"
```

默认参数为：

```text
Device: MSPM0G3507
Interface: SWD
Speed: 100 kHz
Image: Debug/DianSai_MSPM0G3507.out
```

只检查脚本、输出文件和 J-Link 命令，不操作硬件：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode/jlink-flash.ps1 -DryRun
```

### 6.3 从 CCS 调试或下载

工程的目标配置为：

```text
targetConfigs/MSPM0G3507.ccxml
```

该配置已选择 SEGGER J-Link 和 MSPM0G3507。使用 CCS 前应先安装与 CCS 兼容的 SEGGER 驱动，然后在 Target Configurations 中测试连接。

## 7. 常见问题

### 找不到 FreeRTOS 引用工程

确认 CCS 工作区中存在名称完全一致的：

```text
freertos_builds_LP_MSPM0G3507_release_ticlang
```

不要只复制主工程，也不要随意修改引用工程的 `.project` 中的工程名。

### SysConfig 报产品或版本缺失

确认安装并被 CCS 识别的是：

```text
mspm0_sdk@2.11.00.07
sysconfig@1.28.0
```

不要直接修改生成的 `ti_msp_dl_config.c/.h` 来绕过 SysConfig 错误。

### VS Code 中头文件有红色波浪线，但 CCS 可以编译

这是 IntelliSense 路径问题。检查 `.vscode/c_cpp_properties.json` 中的编译器、SDK 和 FreeRTOS 引用工程路径，然后执行 `C/C++: Reset IntelliSense Database`。

### 构建成功但 CCS/J-Link 提示无法打开 `.out`

确认输出文件实际存在：

```text
Debug/DianSai_MSPM0G3507.out
```

工程名和输出名必须保持为 `DianSai_MSPM0G3507`。如果没有该文件，应先解决编译或链接错误，不能继续烧录旧文件。

### J-Link 找到 SW-DP，但无法找到 Cortex-M0+

依次检查：

1. 目标板供电、VTref 和共地。
2. SWDIO、SWCLK 是否接反或接触不良。
3. 将 SWD 速度保持为 `100 kHz`。
4. 连接 `nRESET`，尝试复位下连接。
5. 退出所有可能占用 J-Link 的 CCS 调试会话和 J-Link Commander。

如果 J-Link 提示对 MSPM0 执行 factory reset，只有在确认允许擦除芯片程序、用户数据和 NONMAIN 配置后才能同意。该操作不可作为普通烧录步骤反复执行。

## 8. 工程目录

```text
application/     应用层、机器人和底盘逻辑
bsp/             MSPM0 外设抽象与板级驱动
module/          电机、编码器、IMU、OLED、按键、蜂鸣器等功能模块
freertos/        主工程使用的启动文件
targetConfigs/   CCS/J-Link 目标配置
.vscode/         VS Code 编译、烧录和 IntelliSense 配置
DianSai_MSPM0G3507.syscfg  SysConfig 外设配置源文件
main.c           芯片启动与系统初始化入口
main_freertos.c  FreeRTOS 内核对象和调度器启动
```

## 9. 交接前检查清单

- [ ] 主工程和 FreeRTOS 引用工程已一起打包
- [ ] 未包含 `Debug/`、`Release/` 等本机构建产物
- [ ] 接收方已安装 CCS、MSPM0 SDK、SysConfig 和 TI Arm Clang
- [ ] CCS 工作区中能看到两个工程
- [ ] `.vscode/ccs-build.ps1` 已改为接收方的 CCS 路径
- [ ] `.vscode/c_cpp_properties.json` 已改为接收方的工具链和 SDK 路径
- [ ] CCS 编译后能生成 `Debug/DianSai_MSPM0G3507.out`
- [ ] J-Link 的 VTref、GND、SWDIO、SWCLK 和可选 nRESET 已正确连接
- [ ] 烧录前已确认没有其他程序占用 J-Link

## 10. 修改约定

- MSPM0 外设优先通过 `DianSai_MSPM0G3507.syscfg` 配置。
- 使用 SysConfig 生成的外设实例时包含 `ti_msp_dl_config.h`。
- 新增底层驱动优先使用 TI DriverLib，不引入 STM32 HAL 或 STM32 芯片头文件。
- `mspm0_hal_compat.h` 只用于兼容现有模块中的 HAL 命名习惯，不替代 SysConfig 和 TI DriverLib。
- 提交代码时不要提交 `Debug/`、`.out`、`.map` 等生成文件。
