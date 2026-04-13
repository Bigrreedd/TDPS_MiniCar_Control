# UESTCHN3018 TDPS 微缩车模控制程序

本仓库用于 UESTCHN3018 Team Design Project and Skills 课程的微缩车模（Patio 赛道）底层 C 语言控制程序的团队协同开发。
仓库地址：https://github.com/Bigrreedd/TDPS_MiniCar_Control.git

为了保证 5 人团队的高效协作与代码安全，请所有成员严格遵守本指南中的 **VS Code + Git** 工作流操作规范。

---

## 一、 环境准备 (Environment Setup)

在开始编写微缩车模代码之前，请团队所有成员务必严格按照以下步骤配置本地开发环境：

### 1. 安装 Git 版本控制引擎
* **下载地址**：前往 [Git 官方网站](https://git-scm.com/downloads) 下载对应操作系统的最新 64 位版本。
* **安装注意**：安装过程中一路点击 "Next" 即可。在选择默认编辑器时，建议在下拉菜单中选择 "Use Visual Studio Code as Git's default editor"。请确保 Git 被成功添加到了系统的环境变量 (PATH) 中。

### 2. 安装 VS Code (代码编写与版本管理主战平台)
* **下载地址**：前往 [Visual Studio Code 官网](https://code.visualstudio.com/) 下载。
* **必装插件 (Extensions)**：打开 VS Code，在左侧扩展市场安装以下 3 个官方插件：
    1.  **C/C++ (Microsoft)**：提供底层 C 语言的语法高亮、代码跳转与智能提示 (IntelliSense)。
    2.  **GitLens — Git supercharged**：工程级查错神器，可在代码行末尾直接显示该行代码是谁在何时修改的（精准责任追踪）。
    3.  **Makefile Tools (Microsoft)**：由于我们已在云端配置了 GCC 自动化测试，此插件可解析根目录下的 Makefile，自动补全库路径，消除本地代码的红色波浪线误报。

### 3. 安装 Keil MDK (物理编译与烧录工具)
* **版本要求**：建议使用 Keil uVision5 版本。
* **芯片包依赖 (Device Family Pack)**：确保已通过 Pack Installer 安装了 `Keil.STM32F1xx_DFP` 支持包，否则无法打开和编译本工程的 `.uvprojx` 文件。

---

## 二、 初次获取代码（Clone）

*此操作每台电脑仅需执行一次。这会将云端完整的 Git 数据库及所有历史快照物理下载到你的本地硬盘。*

**⚠️ 前置极其重要条件**：本仓库为**私有仓库 (Private Repository)**。在执行以下步骤前，请务必先登录你的 GitHub 账号，并在网页右上角的通知 (Notifications) 铃铛图标或绑定的邮箱中，**点击接受 (Accept Invitation)** 仓库协作邀请。未接受邀请直接克隆将报 `404 Not Found` 权限拒绝错误。

### 详细操作步骤：

1.  **启动 VS Code**：打开一个干净的 VS Code 界面。
2.  **调出命令面板**：按下键盘快捷键 `Ctrl + Shift + P` (Windows) 或 `Cmd + Shift + P` (Mac)。
3.  **输入克隆指令**：在弹出的顶部搜索框中，输入 `Git: Clone`（或中文界面下的 `Git: 克隆`），并点击下拉列表选中该行。
4.  **输入仓库地址**：在提示框中精准粘贴本团队的云端仓库 URL：
    `https://github.com/Bigrreedd/TDPS_MiniCar_Control.git`
    粘贴后按回车键 (`Enter`)。
5.  **GitHub 身份验证 (Authentication)**：
    * 如果你是第一次在电脑上使用 Git 连接 GitHub，VS Code 或系统弹窗会要求你进行身份验证。
    * 请选择 **"Sign in with your browser" (使用浏览器登录)**，并在弹出的网页中点击 **"Authorize GitCredentialManager"** 完成授权绑定。
6.  **选择本地存储路径 (红线警告)**：
    * 在弹出的文件资源管理器中，选择一个本地文件夹来存放整个车模工程。
    * **⛔ 绝对禁止包含中文或空格的路径！** （例如：`D:\我的大学\大三下\微缩车模` 是**严重违规**的，这会导致 Keil 底层链接器彻底罢工报错）。
    * **✅ 标准规范路径示例**：`D:\Workspace\UESTCHN3018\TDPS_Project`。
7.  **打开工程工作区**：等待右下角进度条跑完（下载时间取决于网速）。下载完成后，VS Code 右下角会弹出提示，直接点击 **"Open" (打开)**。此时，左侧资源管理器会显示所有 `.c`, `.h` 及 `.uvprojx` 文件，代码接入完成！
---

## 三、 日常协作核心工作流 (The Core Workflow)



为了保证底层 C 语言代码数据库的线性一致性，每次开始编写代码或准备共享代码时，全队 5 人必须严格遵循以下单向数据流操作：

### 1. 编写前必做：拉取同步 (Pull)
* **技术本质**：执行 `git fetch`（从云端下载最新的 Blob、Tree 和 Commit 对象）并触发 `git merge`（将本地 `HEAD` 指针向前推进或进行合并）。
* **工程目的**：强制本地工作区与 GitHub 远端数据库对齐，从物理层面上最大程度降低后续的合并冲突概率。
* **操作指令**：
  * 在 VS Code 左侧的 **源代码管理 (Source Control)** 面板中。
  * 点击面板顶部标题栏右侧的 `...` (更多操作) -> 选择 **拉取 (Pull)**。
  * *红线警告：每天到达实验室打开电脑的第一件事，必须是执行 Pull 操作。*

### 2. 本地物理修改 (Modify)
* 在 Keil MDK 或 VS Code 中正常修改 `.c`, `.h` 或 `.uvprojx` 文件。
* **状态同步**：修改完成后，务必在编辑器中按 `Ctrl + S` 保存物理文件，VS Code 底层的 Git 引擎才会重新计算文件的 SHA-1 哈希值并识别到 `(Modified)` 状态。

### 3. 暂存与提交 (Stage & Commit)
* **技术本质**：将工作区的变更写入 `.git/index`（暂存区），随后生成永久的 Commit 对象（快照）并推进本地指针。
* **操作步骤**：
  1. 审查更改：在 **更改 (Changes)** 列表中，点击文件名查看差异视图 (Diff)，确认没有误删关键代码。
  2. 暂存文件：点击待同步文件右侧的 `+` 号（底层等同于 `git add <file>`）。
  3. 规范化提交：在顶部的消息框中输入符合语义化版本规范的提交说明。
  4. 生成快照：点击 **提交 (Commit)**。
* **团队 Commit 命名规范 (Semantic Commits)**：
  必须采用 `<类型>: <具体修改的模块及技术原因>` 的格式：
  * `feat:` 新增功能（例：`feat: 增加 LoRa 串口通信初始化函数`）
  * `fix:` 修复逻辑或语法错误（例：`fix: 修正光电阵列归一化分母为0的致命错误`）
  * `docs:` 文档或注释修改（例：`docs: 更新电机 PWM 占空比计算公式说明`）
  * `refactor:` 代码重构，不改变行为（例：`refactor: 提取 PID 计算部分为独立函数`）

### 4. 推送至云端与 CI 触发 (Push & CI Check)
* **技术本质**：将本地新增的 Commit 对象打包，通过 HTTPS/SSH 协议传输至 GitHub 服务器，并更新远端 `origin` 指针。
* **操作步骤**：点击面板中的蓝色按钮 **同步更改 (Sync Changes)**。
* **⚠️ CI 自动化安检 (Continuous Integration)**：
  * 代码推送后，GitHub Actions 会在云端 Linux 服务器上自动拉取并执行 `make all`。
  * 请前往 GitHub 仓库主页查看提交记录旁边的状态指示灯：
    * **绿勾 ✅**：编译通过，代码安全。
    * **红叉 ❌**：存在语法错误或缺少头文件。**提交者必须立刻在本地修复错误并重新提交，其他队员看到红叉时绝对禁止执行 Pull 操作！**

---

## 四、 团队分支与防冲突策略 (Branching & Conflict Resolution)



5 人同时在单片机工程中修改代码（尤其是 `main.c` 和 `stm32f10x_it.c` 中断文件）极易造成底层数据覆写。本工程严格执行“主干保护与功能隔离”策略：

### 1. 绝对的功能隔离 (Feature Branching)
任何独立模块（如 Task1 循迹、Task3 雷达扫描）的开发，**严禁**直接在 `main` 分支上进行。`main` 分支必须永远保持 100% 能够通过编译且能在赛道上稳定运行的状态。
* **创建工作指针 (Create Branch)**：点击 VS Code 左下角的 `main` 按钮 -> 选择 **创建新分支**。
  * **命名规范**：`姓名缩写/功能名`（例：`ZJ/feature-radar`, `LHX/lora-task`）。
* **云端备份 (Publish Branch)**：点击左下角的云朵图标，将此指针同步至 GitHub。
* **合并准入 (Merge to Main)**：
  1. 在实车硬件上测试无逻辑 Bug。
  2. 确认云端 CI 编译状态为 **绿勾 ✅**。
  3. 切换回 `main` 分支，按 `Ctrl+Shift+P` 输入 `Git: Merge Branch`，选择你的功能分支执行合并，并最终 Push 到云端。

### 2. 解决合并冲突的底层方法 (Resolve Merge Conflicts)


当两名队员修改了同一个 `.c` 文件的同一行代码并尝试合并时，Git 会停止合并动作，在文件中插入标准的冲突定界符。

**冲突表现示例 (如 PID 调参冲突)**：
```c
<<<<<<< HEAD (当前更改 - 你本地 main 分支的代码)
float Kp = 12.5;  // 你认为小车应该用这个参数
=======
float Kp = 15.0;  // 队友推送的新参数
>>>>>>> ZJ/feature-radar (传入的更改 - 试图合并进来的代码)
```
---

## 五、 ⚠️ Keil 工程特有协作红线 (Keil-Specific Collaboration Red Lines)

Keil MDK 的底层构建逻辑与标准的开源代码编辑器不同。在执行“Build”操作时，底层的 ARMCC 编译器和汇编器会生成海量的衍生数据。如果不进行严格的物理隔离，这些频繁变动的二进制数据将瞬间摧毁 Git 数据库的线性结构。全队必须绝对遵守以下 3 条工程红线：

### 1. 绝对禁止追踪编译产物与中间文件 (Prohibit Intermediate Files)
* **技术本质**：Keil 编译时会在 `Objects/` 和 `Listings/` 目录下生成数十个 `.o` (目标对象文件), `.d` (依赖关系图), `.crf` (交叉引用文件), 以及最终的 `.map` 和 `.hex` 等物理产物。这些二进制文件每次编译都会发生基于时间戳和内存地址的哈希重组。
* **工程危害**：如果将这些文件推送到 GitHub，不仅会导致云端仓库体积呈指数级膨胀（Bloat），更会在队友间引发 100% 无法通过纯文本比对来解决的合并冲突（Binary Merge Conflict）。
* **操作红线**：本仓库已在根目录部署了严格的 `.gitignore` 规则。在 VS Code 左侧的“源代码管理”面板中，**你绝对不应该看到任何位于 `Objects/` 或 `Listings/` 文件夹下的文件变动**。如果出现了，说明你的本地 Git 索引配置异常，请立刻停止 Commit 动作，并在团队群内联系仓库管理员执行底层索引清理（`git rm -r --cached .`）。

### 2. 绑定提交项目树配置文件 (Correctly Commit `.uvprojx`)
* **技术本质**：`Project.uvprojx` 是一个基于 XML 语法的工程架构记录文件。它向底层的 Keil 编译器指明了左侧 Project Tree（工程树）中包含了哪些具体的 `.c` 文件，以及它们对应的头文件搜索路径（Include Paths）。
* **工程危害**：假设你为了开发雷达扫描功能，在硬盘上新建了 `Radar.c`，并在 Keil 软件界面内通过 `Add Existing Files to Group` 将其引入了工程。如果你只向 Git 暂存（Stage）了 `Radar.c` 而遗漏了 `Project.uvprojx`，队友执行 Pull 后，他的物理硬盘上会拥有 `Radar.c` 代码，但他的 Keil 工程树里**依然没有这个文件**。底层的链接器（Linker）将完全无视该文件，导致工程编译失败。
* **操作红线**：只要你在 Keil 界面中执行了**添加文件、移除文件、或修改了魔术棒 (Options for Target) 里的任何配置（例如开启 C99 模式、修改全局宏定义）**，`Project.uvprojx` 都会发生变动。此时，你必须将 `.uvprojx` 文件与你新增的 C 语言源码文件**一起点击 `+` 号暂存，并将它们打包在同一个 Commit 中推送**。

### 3. 绝对禁止提交本地调试配置文件 (Strictly Prohibit `.uvoptx`)
* **技术本质**：`.uvoptx`（uVision Option XML）是单机硬件调试状态的快照文件。它物理存储了你当前这台电脑上高度定制化的硬件与 UI 信息，例如：ST-Link/DAP-Link 仿真器的唯一序列号、底层下载速率设置、编辑器上次关闭时光标所在的具体行数、以及你设置的每一个物理断点（Breakpoint）的具体内存地址。
* **工程危害**：如果该文件被推送到 `main` 分支，队友拉取后，他本地真实的仿真器配置将被你的 XML 数据强行覆盖。这极易触发 `No Debug Unit Found`（未检测到下载器）的底层硬件通信错误，直接导致队友的实车烧录进度全面停滞。
* **操作红线**：该文件已被 `.gitignore` 彻底屏蔽。如果你在极端情况下发现 VS Code 提示包含了 `.uvoptx` 的更改，**绝对不允许将其暂存或提交**。必须让其保持在未追踪 (Untracked) 或被忽略的状态。

---

## 六、 云端自动化代码审查系统 (Cloud CI/CD Pipeline)



本工程已在 GitHub 部署基于 Actions 的持续集成 (Continuous Integration, CI) 系统。该系统完全脱离本地 Keil 闭源生态，使用跨平台的开源标准编译工具链对全量代码进行自动化、无缓存验证。

### 1. 底层运行机制 (Execution Mechanism)
每次团队成员向 `main` 分支执行 Push 操作，或发起合并请求 (Pull Request) 时，GitHub 将自动触发以下底层技术流程：
* **环境初始化**：云端启动一台全新分配的、纯净的 Ubuntu Linux 虚拟机实例。
* **依赖拉取**：通过系统的 `apt` 包管理器，动态安装 `arm-none-eabi-gcc` 交叉编译工具链。
* **执行全量编译**：系统解析工程根目录下的 `Makefile`，调用专属的 GCC 启动文件 (`startup_stm32f10x_md_gcc.s`) 和内存映射链接脚本 (`STM32F103XB_FLASH.ld`)，将所有 `.c` 文件重新编译、汇编并链接为目标文件。

### 2. 工程部署核心价值 (Engineering Significance)
在 5 人协同的微缩车模开发中，此系统用于解决底层硬件开发中极其致命的数据流同步问题：
* **消除本地缓存污染 (Clean Room Build)**：Keil 编译器存在严重的中间文件缓存机制。本地编译通过的代码，可能在文件系统层面已经缺失了 `.h` 头文件引用。云端 CI 每次都在零缓存环境中进行构建，可 100% 验证当前 Git 提交节点的文件依赖完整性。
* **强制语法标准规范 (Strict Syntax Compliance)**：现代 GCC 编译器对底层 C 语言标准（如架构级寄存器约束、变量声明作用域）的检查，远比早期版本的 ARMCC 严苛。通过云端 GCC 审查的代码，消除了编译器方言，具备极高的可靠性。
* **提供客观的协作阻断机制 (Sync Blocking)**：当代码存在语法错误时，系统产生客观的失败状态标记，从物理流程上警告其他节点（团队成员）停止下行同步，防止错误源码在团队内部发生二次级联扩散。

### 3. 状态判定与错误响应规范 (Status Check & Handling)
每次推送代码后，提交者必须前往仓库主页确认 Commit 旁的状态指示灯：
* **绿勾 ✅ (Success)**：全量代码 0 语法错误，依赖树完整。代码处于安全状态，全队可随时执行 Pull 同步。
* **红叉 ❌ (Failure)**：编译或链接发生致命错误（Fatal Error）。
    * **强制阻断**：**在仓库主状态恢复为绿勾 ✅ 之前，团队所有其他成员绝对禁止执行 Pull 拉取操作！**
    * **处理标准**：代码提交者必须点击 `Details` 查看报错日志，精准定位（通常为缺失头文件路径 `-I` 或标准库链接错误），在本地彻底修复后，重新执行暂存与提交流程，直至状态翻转为绿勾。
