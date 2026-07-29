# 在 VS Code 中编译和烧录 MSPM0G3507

## 本机工具路径

```text
CCS CLI:
C:\TI\ccs2100\ccs\eclipse\ccs-server-cli.bat

TI Arm Clang:
C:\TI\ccs2100\ccs\tools\compiler\ti-cgt-armllvm_5.1.1.LTS\bin\tiarmclang.exe

MSPM0 SDK:
C:\TI\mspm0_sdk_2_11_00_07

J-Link Commander:
D:\JLINK\JLink_V850\JLink.exe
```

CCS 命令行使用独立工作区：

```text
%LOCALAPPDATA%\TI\CCS\workspaces\DianSai_MSPM0G3507-cli
```

它不会和 CCS 图形界面的工作区互相占用，也不会写入工程源码目录。

## 首次使用

使用 VS Code 打开：

```text
D:\DianSai\m0\m0\DianSai_MSPM0G3507
```

确认同级目录中存在 FreeRTOS 引用工程：

```text
D:\DianSai\m0\m0\freertos_builds_LP_MSPM0G3507_release_ticlang
```

打开 VS Code 的 `Terminal -> Run Task`，可以执行以下任务：

| 任务 | 作用 |
| --- | --- |
| `CCS: Build Debug` | 增量编译 Debug |
| `CCS: Full Build Debug` | 完整重新编译 Debug |
| `J-Link: Flash Debug` | 直接烧录现有 Debug 输出 |
| `J-Link: Build and Flash Debug` | 先增量编译，再烧录 |

默认构建快捷键 `Ctrl+Shift+B` 对应 `CCS: Build Debug`。

首次编译时，脚本会把主工程及同级 FreeRTOS 引用工程导入 CCS 命令行工作区。编译成功后输出：

```text
Debug\DianSai_MSPM0G3507.out
```

## 命令行等价操作

增量编译：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\ccs-build.ps1 -BuildType incremental
```

完整编译：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\ccs-build.ps1 -BuildType full
```

烧录：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\jlink-flash.ps1
```

只检查输出文件、J-Link 路径和烧录命令，不操作硬件：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\jlink-flash.ps1 -DryRun
```

## 烧录连接

```text
J-Link VTref -> 开发板 3.3 V
J-Link GND   -> 开发板 GND
J-Link SWDIO -> MSPM0G3507 PA19
J-Link SWCLK -> MSPM0G3507 PA20
J-Link RESET -> 建议连接 nRESET
```

默认参数：

```text
Device: MSPM0G3507
Interface: SWD
Speed: 100 kHz
```

烧录前关闭可能占用同一个 J-Link 的 CCS 调试会话和 J-Link Commander。
