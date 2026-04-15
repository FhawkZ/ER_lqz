# Git 统一仓库多设备协作说明

本文档适用于同一个远程仓库会被多台设备、多人同时修改的场景，目标是：

- 保持远程分支历史清晰可追溯
- 降低冲突发生概率
- 在冲突发生后快速、安全地恢复协作

## 1. 协作基础约定

建议统一以下规则：

- 远程仓库地址统一为 `origin`
- 主分支统一为 `main`（如你们使用 `master`，将文中 `main` 替换为 `master`）
- 每人/每个任务使用独立功能分支，例如：
  - `feature/alice-data-pipeline`
  - `fix/bob-franka-launch`
- 禁止直接在 `main` 上日常开发，`main` 只接收经过审核的合并

建议每台设备都先完成以下一次性配置：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
git config --global pull.rebase false
git config --global fetch.prune true
```

说明：

- `pull.rebase false` 表示默认使用 merge 方式拉取（更直观，适合团队统一）
- 若团队统一使用 rebase，也可以设为 `true`，但需全员一致

## 2. 新设备接入流程

在新设备上首次接入：

```bash
git clone <仓库URL>
cd ER_lqz
git checkout main
git pull origin main
```

开始开发前，先从最新主分支切出任务分支：

```bash
git checkout -b feature/<你的任务名> main
```

## 3. 每日同步标准流程（推荐）

以下流程适用于“多人+多设备”日常协作，建议每次开始工作都执行：

1. 获取远程最新状态
2. 更新本地主分支
3. 再把主分支变更同步到你的任务分支

示例命令：

```bash
# 1) 更新远程引用
git fetch origin

# 2) 切到主分支并更新
git checkout main
git pull origin main

# 3) 回到你的分支并同步主分支内容（merge 方案）
git checkout feature/<你的任务名>
git merge main
```

如果你们团队使用 rebase，则第 3 步改为：

```bash
git rebase main
```

## 4. 提交与推送规范

在本地完成一个小闭环后再提交，避免“大而杂”的提交：

```bash
git add <文件或目录>
git commit -m "feat: 简述本次改动目的"
git push -u origin feature/<你的任务名>
```

建议：

- 一个 commit 只做一件事（便于回滚和排查）
- 提交信息写“为什么改”，而不只是“改了什么”
- 推送前先执行一次 `git fetch origin`，确认远程没有新变化

## 5. 多人修改同一仓库时的同步策略

### 场景 A：多人改不同文件

通常无冲突，按标准流程同步即可。

### 场景 B：多人改同一文件但不同区域

可能自动合并成功，但建议合并后手动检查关键文件是否符合预期，并执行基本测试。

### 场景 C：多人改同一文件同一区域（高概率冲突）

必须人工处理冲突。建议先沟通“最终以谁的逻辑为准”，再进行解决，避免反复覆盖。

## 6. 冲突处理标准操作

当执行 `git merge main` 或 `git pull` 后出现冲突：

### 6.1 查看冲突文件

```bash
git status
```

会看到类似提示：`both modified: <file>`

### 6.2 打开文件，处理冲突标记

冲突内容示例：

```text
<<<<<<< HEAD
你的本地内容
=======
远程（或被合并分支）内容
>>>>>>> main
```

处理方式：

- 保留你自己的版本
- 保留对方版本
- 合并成一个新版本（最常见）

处理后请删除 `<<<<<<<`、`=======`、`>>>>>>>` 标记。

### 6.3 标记为已解决并完成合并

```bash
git add <冲突文件>
git commit -m "merge: resolve conflict with main"
```

若是 `rebase` 过程中冲突：

```bash
git add <冲突文件>
git rebase --continue
```

### 6.4 回归验证

至少执行：

- 项目能启动
- 关键脚本可运行（如 `scripts/*.sh`）
- 本次改动相关功能不报错

验证通过后再推送：

```bash
git push
```

## 7. 常见冲突预防建议

- 高频同步：每次开始开发前先同步，不要长时间不拉取
- 小步提交：减少单次变更体积，降低冲突面
- 文件分工：提前约定负责人，减少多人同时改同一模块
- 先沟通再改：对核心文件（启动脚本、配置、公共接口）先在群里对齐
- 锁定关键时段：发布前或联调时，指定一个集成人员统一合并

## 8. 建议的团队协作模型（简化版）

推荐使用“主分支 + 功能分支 + PR/MR 审核”：

1. 从 `main` 拉新分支开发
2. 分支推送到远程
3. 发起 PR/MR
4. 至少 1 人 Review
5. CI（如有）通过后再合并到 `main`

好处：

- 降低坏提交直接进入主分支的风险
- 出问题可快速定位到具体分支/提交
- 新设备拉取 `main` 时更稳定

## 9. 快速命令清单

```bash
# 查看当前状态
git status

# 拉取远程更新（不合并）
git fetch origin

# 更新主分支
git checkout main && git pull origin main

# 将主分支同步到当前功能分支（merge）
git merge main

# 查看提交历史
git log --oneline --graph --decorate -20

# 查看当前分支与远程差异
git branch -vv
```

## 10. 故障应对（简版）

- **误操作但未提交**：优先先备份改动（复制文件），再恢复
- **冲突解决后行为异常**：对照冲突文件历史，逐段回看变更来源
- **分支混乱不确定怎么处理**：先暂停 push，找团队内熟悉 Git 的同学一起确认后再操作

---

如果你们后续希望，我可以基于你们当前仓库再补一份“严格执行版流程”（包含分支命名、提交模板、发布前检查清单、冲突责任人机制），方便团队直接照着执行。
