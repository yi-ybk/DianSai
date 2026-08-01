# 在 VS Code 中编译和烧录 MSPM0G3507

## 本机工具路径

CCS CLI 默认从 `CCS_ROOT`、`PATH` 和各本地磁盘的常见 `ccs*`
安装目录自动发现，也可以通过 `-CcsCli` 显式指定。

J-Link Commander 默认从 SEGGER 注册表安装目录自动发现，也可以通过
`-JLinkExe` 显式指定。

CCS 命令行使用工程同级的独立工作区：

```text
<工程父目录>\.ccs-cli-workspace-compile-check
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
| `J-Link: 读取循迹RAM` | 不复位、不重烧，读取循迹轨迹并生成 CSV/JSON |
| `MSPM0: 一键编译并烧录` | 完整编译成功后自动烧录、校验并复位 |

默认构建快捷键 `Ctrl+Shift+B` 对应 `MSPM0: 一键编译并烧录`。

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

一键编译并烧录：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\mspm0-build-flash.ps1
```

烧录：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\jlink-flash.ps1
```

只检查输出文件、J-Link 路径和烧录命令，不操作硬件：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\jlink-flash.ps1 -DryRun
```

读取循迹 RAM（运行结束后保持开发板供电，且不要按复位键）：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .vscode\read-track-ram.ps1
```

脚本只暂停 CPU、读取 `track_trace_store` 和 `track_debug_state`，随后恢复运行；
不会执行复位或重新烧录。输出保存在 `Debug\track-trace-时间戳.csv/json`。

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
