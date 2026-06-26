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
- 本项目为嵌入式项目，芯片为stm32f407vet6
- EIDE Build 实现：
当前使用 cl.eide-3.27.0。插件 Build 按钮流程为:
从 .eide/eide.yml读取目标配置
生成 builder.params
通过 VS Code Shell Task 调用插件内置的 unify_builder.exe
unify_builder 再调用 AC5 工具链完成编译和链接

后续编译命令
EIDE 生成最新 builder.params 后，可以直接执行增量编译：
& 'C:\Users\tianxuan\.vscode\extensions\cl.eide-3.27.0\res\tools\win32\unify_builder\unify_builder.exe' `
  -p 'D:\STM32Hal\DianSai\build\DianSai\builder.params'

完整重编译追加：
--rebuild

仅演练或重新生成编译数据库可追加：
--dry-run
--only-dump-compilerdb

构建诊断文件位于：
build/DianSai/compiler.log
build/DianSai/unify_builder.log
build/DianSai/compile_commands.json

若因 builder.params 尚未纳入源码而无法进行编译验证，你自行在 EIDE 项目树中补充

### 要求
除非用户要求，不要修改代码文件中的注释，除非原有注释与现有代码的语义不符，则重新按照工程原有的注释风格修改、添加、删除注释
新增、修改代码或者模块时，要参考工程中原有的注释风格修改、添加、删除注释