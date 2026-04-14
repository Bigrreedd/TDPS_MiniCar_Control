# UESTCHN3018 TDPS 微缩车模控制程序

本仓库用于 **UESTCHN3018 Team Design Project and Skills** 课程的微缩车模（Patio 赛道）底层 C 语言控制程序的团队协同开发。

**仓库地址：** `https://github.com/Bigrreedd/TDPS_MiniCar_Control.git`

为了保证 5 人团队的高效协作与代码安全，请所有成员严格遵守本指南中的 **VS Code + Git** 工作流操作规范。

---

## 一、 环境准备 (Environment Setup)

在开始编写微缩车模代码之前，请团队所有成员务必严格按照以下步骤配置本地开发环境：

### 1. 安装 Git 版本控制引擎
* **下载地址：** 前往 [Git 官方网站](https://git-scm.com/) 下载对应操作系统的最新 64 位版本。
* **安装注意：** 安装过程中一路点击 "Next" 即可。在选择默认编辑器时，建议在下拉菜单中选择 **"Use Visual Studio Code as Git's default editor"**。请确保 Git 被成功添加到了系统的环境变量 (PATH) 中。

### 2. 安装 VS Code (代码编写与版本管理主战平台)
* **下载地址：** 前往 [Visual Studio Code 官网](https://code.visualstudio.com/) 下载。
* **必装插件 (Extensions)：** 打开 VS Code，在左侧扩展市场安装以下 3 个官方插件：
    * **C/C++ (Microsoft)：** 提供底层 C 语言的语法高亮、代码跳转与智能提示 (IntelliSense)。
    * **GitLens — Git supercharged：** 工程级查错神器，可在代码行末尾直接显示该行代码是谁在何时修改的（精准责任追踪）。
    * **Makefile Tools (Microsoft)：** 由于我们已在云端配置了 GCC 自动化测试，此插件可解析根目录下的 Makefile，自动补全库路径，消除本地代码的红色波浪线误报。

### 3. 安装 Keil MDK (物理编译与烧录工具)
* **版本要求：** 建议使用 Keil uVision5 版本。
* **芯片包依赖 (Device Family Pack)：** 确保已通过 Pack Installer 安装了 `Keil.STM32F1xx_DFP` 支持包，否则无法打开和编译本工程的 `.uvprojx` 文件。

---

## 二、 初次获取代码（Clone）

此操作每台电脑仅需执行一次。这会将云端完整的 Git 数据库及所有历史快照物理下载到你的本地硬盘。

> ⚠️ **前置极其重要条件：** 本仓库为私有仓库 (Private Repository)。在执行以下步骤前，请务必先登录你的 GitHub 账号，并在网页右上角的通知 (Notifications) 铃铛图标或绑定的邮箱中，点击**接受 (Accept Invitation)** 仓库协作邀请。未接受邀请直接克隆将报 `404 Not Found` 权限拒绝错误。

**详细操作步骤：**

1.  **启动 VS Code：** 打开一个干净的 VS Code 界面。
2.  **调出命令面板：** 按下键盘快捷键 `Ctrl + Shift + P` (Windows) 或 `Cmd + Shift + P` (Mac)。
3.  **输入克隆指令：** 在弹出的顶部搜索框中，输入 `Git: Clone`（或中文界面下的 `Git: 克隆`），并点击下拉列表选中该行。
4.  **输入仓库地址：** 在提示框中精准粘贴本团队的云端仓库 URL： `https://github.com/Bigrreedd/TDPS_MiniCar_Control.git` 粘贴后按回车键 (Enter)。
5.  **GitHub 身份验证 (Authentication)：** 如果你是第一次在电脑上使用 Git 连接 GitHub，VS Code 或系统弹窗会要求你进行身份验证。请选择 **"Sign in with your browser"** (使用浏览器登录)，并在弹出的网页中点击 **"Authorize GitCredentialManager"** 完成授权绑定。
6.  **选择本地存储路径 (红线警告)：** 在弹出的文件资源管理器中，选择一个本地文件夹来存放整个车模工程。
    * ⛔ **绝对禁止包含中文或空格的路径！** （例如：`D:\我的大学\大三下\微缩车模` 是严重违规的，这会导致 Keil 底层链接器彻底罢工报错）。
    * ✅ **标准规范路径示例：** `D:\Workspace\UESTCHN3018\TDPS_Project`。
7.  **打开工程工作区：** 等待右下角进度条跑完。下载完成后，VS Code 右下角会弹出提示，直接点击 **"Open"** (打开)。此时，左侧资源管理器会显示所有 `.c`, `.h` 及 `.uvprojx` 文件，代码接入完成！

---

## 三、 日常协作核心工作流 (The Core Workflow)

为了保证底层 C 语言代码数据库的线性一致性，每次开始编写代码或准备共享代码时，全队 5 人必须严格遵循以下单向数据流操作：

### 1. 编写前必做：拉取同步 (Pull)
* **技术本质：** 执行 `git fetch`（从云端下载最新的 Blob、Tree 和 Commit 对象）并触发 `git merge`（将本地 `HEAD` 指针向前推进或进行合并）。
* **工程目的：** 强制本地工作区与 GitHub 远端数据库对齐，从物理层面上最大程度降低后续的合并冲突概率。
* **操作指令：** 在 VS Code 左侧的 **源代码管理 (Source Control)** 面板中，点击面板顶部标题栏右侧的 `...` (更多操作) -> 选择 **拉取 (Pull)**。
* 🔴 **红线警告：** 每天到达实验室打开电脑的第一件事，必须是执行 Pull 操作。

### 2. 本地物理修改 (Modify)
* 在 Keil MDK 或 VS Code 中正常修改 `.c`, `.h` 或 `.uvprojx` 文件。
* **状态同步：** 修改完成后，务必在编辑器中按 `Ctrl + S` 保存物理文件，VS Code 底层的 Git 引擎才会重新计算文件的 SHA-1 哈希值并识别到 **(Modified)** 状态。

### 3. 暂存与提交 (Stage & Commit)
* **技术本质：** 将工作区的变更写入 `.git/index`（暂存区），随后生成永久的 Commit 对象（快照）并推进本地指针。
* **操作步骤：**
    1.  **审查更改：** 在 **更改 (Changes)** 列表中，点击文件名查看差异视图 (Diff)，确认没有误删关键代码。
    2.  **暂存文件：** 点击待同步文件右侧的 `+` 号（底层等同于 `git add <file>`）。
    3.  **规范化提交：** 在顶部的消息框中输入符合语义化版本规范的提交说明。
    4.  **生成快照：** 点击 **提交 (Commit)**。
* **团队 Commit 命名规范 (Semantic Commits)：** 必须采用 `<类型>: <具体修改的模块及技术原因>` 的格式：
    * `feat:` 新增功能（例：`feat: 增加 LoRa 串口通信初始化函数`）
    * `fix:` 修复逻辑或语法错误（例：`fix: 修正光电阵列归一化分母为0的致命错误`）
    * `docs:` 文档或注释修改（例：`docs: 更新电机 PWM 占空比计算公式说明`）
    * `refactor:` 代码重构，不改变行为（例：`refactor: 提取 PID 计算部分为独立函数`）

### 4. 推送至云端与 CI 触发 (Push & CI Check)
* **技术本质：** 将本地新增的 Commit 对象打包，通过 HTTPS/SSH 协议传输至 GitHub 服务器，并更新远端 `origin` 指针。
* **操作步骤：** 点击面板中的蓝色按钮 **同步更改 (Sync Changes)**。
* ⚠️ **CI 自动化安检 (Continuous Integration)：** 代码推送后，GitHub Actions 会在云端 Linux 服务器上自动拉取并执行 `make all`。请前往 GitHub 仓库主页查看提交记录旁边的状态指示灯：
    * **绿勾 ✅：** 编译通过，代码安全。
    * **红叉 ❌：** 存在语法错误或缺少头文件。提交者必须立刻在本地修复错误并重新提交，其他队员看到红叉时绝对禁止执行 Pull 操作！

---

## 四、 团队分支与防冲突策略 (Branching & Conflict Resolution)

5 人同时在单片机工程中修改代码（尤其是 `main.c` 和 `stm32f10x_it.c` 中断文件）极易造成底层数据覆写。本工程严格执行“主干保护与功能隔离”策略：

### 1. 绝对的功能隔离 (Feature Branching)
任何独立模块（如 Task1 循迹、Task3 雷达扫描）的开发，严禁直接在 `main` 分支上进行。`main` 分支必须永远保持 100% 能够通过编译且能在赛道上稳定运行的状态。
* **创建工作指针 (Create Branch)：** 点击 VS Code 左下角的 `main` 按钮 -> 选择 **创建新分支**。命名规范：`姓名缩写/功能名`（例：`ZJ/feature-radar`, `LHX/lora-task`）。
* **云端备份 (Publish Branch)：** 点击左下角的云朵图标，将此指针同步至 GitHub。
* **合并准入 (Merge to Main)：**
    1.  在实车硬件上测试无逻辑 Bug。
    2.  确认云端 CI 编译状态为 **绿勾 ✅**。
    3.  切换回 `main` 分支，按 `Ctrl+Shift+P` 输入 `Git: Merge Branch`，选择你的功能分支执行合并，并最终 Push 到云端。

### 2. 解决合并冲突的底层方法 (Resolve Merge Conflicts)
当两名队员修改了同一个 `.c` 文件的同一行代码并尝试合并时，Git 会停止合并动作，在文件中插入标准的冲突定界符。

**冲突表现示例 (如 PID 调参冲突)：**
```c
<<<<<<< HEAD (当前更改 - 你本地 main 分支的代码)
float Kp = 12.5;  // 你认为小车应该用这个参数
=======
float Kp = 15.0;  // 队友推送的新参数
>>>>>>> ZJ/feature-radar (传入的更改 - 试图合并进来的代码)