# FTC20827-编码规范

版本：2025.11

---

## 1. 命名约定（必须遵守）

- 包名：全小写，使用反向域名风格。本仓库主包已使用 `org.firstinspires.ftc.teamcode`，子包按功能（`subsystems`, `teleops`, `utils`, `external`）划分。
- 类名：PascalCase（首字母大写），表达单一职责，例如 `Hardwares`, `Shooter`, `MecanumDrive`。
- 接口名：PascalCase，常用能力描述或名词。
- 方法名：动词-名词格式，camelCase，例如 `initialize`, `updatePose`, `setShooter`。
- 字段/变量：camelCase，避免无意义缩写（循环索引除外）。
- 常量：全大写，使用下划线分隔，例如 `MAX_SPEED`。
- 硬件映射名称（HardwareMap 名称）：与 `Hardwares` 中使用的字符串一致（例如 `frontLeft`, `backRight`, `shooterFront`, `intake`），硬件 Map 名称与代码常量或变量命名应一一对应，便于在运行时快速定位。

示例：

- 包：`org.firstinspires.ftc.teamcode.subsystems`
- 类：`Shooter`（文件 `Shooter.java`）
- 硬件常量/变量：`mFrontLeft`, `shooterFront`, `preShooter`

---

## 2. 代码组织（必须遵守）

基于现有结构（见 `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`）：

- 按功能分包：

  - `subsystems/`：机器人各子系统（Shooter, Intake 等）。
  - `teleops/`：遥控 OpMode、TeleOp 入口类（例如 `SingleTeleOp`）。
  - `autos/`：自动OpMode。
  - `utils/`：通用工具类（例如 `MecanumDrive`, `ButtonEx`）。
  - `external/`：第三方或引入的库代码（例如 `PedroPathing`），保持原作者结构尽量不改动以便日后同步更新。

- 每个类一个文件，文件名等于类名（`Shooter.java` 包含 `public class Shooter`）。

- 将硬件映射集中在单一类（目前为 `Hardwares`），不要在多个类中重复读取 HardwareMap 名称字符串，也不要对硬件做非必要的配置。

- 避免超大类：当类超过 ~400 行或承担多种职责时，拆分为更小的子类或 Manager/Controller。

- 对外暴露的子系统应提供清晰的控制接口（如 `startIntake()`, `stopIntake()`, `setShooter()`），而不是直接暴露底层硬件字段给外部修改。

---

## 3. 特别约定（必须遵守）

这一部分针对 OpMode 与硬件交互，结合仓库现有实现（使用 FTCLib CommandScheduler、GamepadEx、Hardwares）：

- OpMode 生命周期：严格遵守 `init`/`start`/`loop`/`stop`（或框架封装的 `initialize`/`onStart`/`run`），不得在主线程执行长阻塞任务。
- 硬件映射：集中使用 `Hardwares` 类进行 HardwareMap 获取与初始化。不要在多个 OpMode 中重复写硬件名称字符串，以免不一致。
- 硬件命名一致性：HardwareMap 名称（XML/phone 配置）应与 `Hardwares` 的字符串保持一致（例如 `frontLeft`, `preShooter` 等）。如果修改了映射名称，须同步更新 `Hardwares` 并在 PR 中说明。
- Telemetry 与频率：避免过于频繁的 telemetry 更新；建议普通状态更新频率 10-20 Hz（每 50-100 ms）以免影响主循环性能。仅在调试阶段增加高频输出，并在 PR 中标注调试代码应删除或受 DEBUG 控制。
- 线程安全：所有直接访问硬件（电机、传感器）的位置应集中并保证线程安全；若在不同线程中使用硬件，需在文档中明确并使用合适的同步/消息传递。
- 超时与失败处理：对机械动作和外设操作设置合理超时和回退策略（例如传感器读取失败应有默认值或安全停机策略）。
- 使用 CommandScheduler：如当前代码所示，推荐将行为封装为命令/子系统并使用 `CommandScheduler` 调度，保持 OpMode 入口薄（只负责初始化和调度）。

---

## 4. PR 流程（建议遵守）

目标：保证变更可审查、可回滚且在合并前通过基础检查。

- 提交信息（commit）：

  - Summary 开头描述 commit 类型，如 `fix`，`feature`
  - Summary 不超过 20 词，描述主要变更内容
  - 示例：`[feature] Add Shooter subsystem with basic control` 。

- 合并前检查项（最低要求）：

  - 相关 OpMode 能在本地构建并在机器人上运行。
  - 代码遵守本规范中的命名与组织约定。
  - 关键逻辑有注释或 PR 描述清楚风险与回退方案。
  - 需要`2 位协作者` 或 `1 位协作者和GitHub Copilot`进行 Code Review。
