# 机器学习规划数据集设想

## 1. 现在的成功数据能不能拿来训练

可以，但要分清楚“训练什么”。

当前这批成功案例，适合先拿来训练下面几类模型：

- `base/lift/pre-insert` 参数建议模型
- `pre-insert` 候选排序模型
- 阶段成功率预测模型
- `arm` / `mobile_arm` 组选择模型
- 是否需要 fallback 的判别模型

当前还不太适合直接训练下面这类模型：

- 端到端输出完整关节轨迹的动作策略
- 直接从场景到控制命令的低层策略
- 高质量离线强化学习策略

原因很简单：

- 现在记录了场景参数、目标位姿、阶段结果和时间
- 但还没有系统记录每个阶段的完整输入状态、候选集、规划结果轨迹、执行反馈轨迹

所以结论是：

- 可以先做“策略选择/候选打分/参数回归”类学习
- 不能假设现在的数据已经足够支撑“学会整套动作”

## 2. 当前已经有的可用字段

当前每个实验 case 已经有：

- 柜体参数
- 目标盒参数
- 目标吸取位姿
- 是否成功
- 最终停在哪个阶段
- 每个阶段耗时
- 原始运行日志
- 失败类型分析

这些字段足够支持：

- 成功/失败二分类
- 不同柜型下的阶段耗时预测
- 哪些柜型更容易失败的统计模型
- 粗粒度的 planner/group 选择模型

## 3. 当前最缺的字段

如果想让成功数据真正能训练出“规划动作”的模型，至少还需要把下面这些结构化记录下来。

### 3.1 Episode 级元数据

- 代码版本
- URDF/SRDF 版本
- 算法版本
- 关键 launch 参数
- 随机 seed

### 3.2 场景输入

- 货架几何参数
- 目标盒几何参数
- clutter 几何参数
- 目标 pose
- 货架入口平面
- 如有点云：原始点云或体素化点云快照

### 3.3 机器人初始状态

- 当前全关节状态
- 当前 base pose
- 当前 raise 高度
- 当前末端 pose
- stow 状态模板

### 3.4 每个阶段的决策输入

以 `SELECT_PRE_INSERT` 为例，应该记录：

- 所有生成的候选 pose
- 每个候选的 retreat / lateral / yaw 参数
- 每个候选是否在柜口外
- 每个候选的 IK 是否成功
- 每个候选的 state validity 是否通过
- 每个候选的 cheap score
- 最终选中了哪个候选

### 3.5 每个阶段的规划输出

- 使用的 group
- 使用的 planner id
- 是否 seeded IK 成功
- 是否用了 fallback
- 规划时间
- 规划成功/失败
- 最终 joint state
- 完整 `JointTrajectory`

### 3.6 执行反馈

- 实际发送给 controller 的 trajectory
- 真实 joint state 时间序列
- 末端实际达到误差
- 是否中途切换候选
- 最终失败原因

### 3.7 负样本

负样本非常重要，至少要记录：

- 哪个候选失败
- 失败在哪一步
- 是 IK 失败、碰撞、超时、`r1` 超限，还是 base motion 超限

没有这些负样本，模型只能学“什么像成功”，很难学“什么一定不要选”。

## 4. 最值得先做的数据集任务

我建议按下面顺序做，不要一上来就追求端到端策略。

### 4.1 `pre-insert` 候选排序

输入：

- 柜体参数
- 目标 pose
- 当前 joint state
- 候选 `pre-insert` 参数

输出：

- 该候选是否最终可达
- 或者该候选的成功分数

这个任务最适合你当前系统，因为你本来就有“候选生成 -> 选一个 -> 规划”的结构。

### 4.2 base / lift 自适应参数回归

输入：

- 柜体尺寸
- 目标位置
- 当前 robot 状态

输出：

- `base_standoff`
- `base_lateral_offset`
- `lift_target_center`
- `pre_insert_offset`

这个任务能直接减少手工调参。

### 4.3 插入阶段 group / fallback 选择

输入：

- `pre-insert` 到位状态
- 当前末端与目标的相对位姿
- 当前 base 偏差
- 当前 arm configuration

输出：

- `arm` 还是 `mobile_arm`
- 先 Cartesian 还是先 OMPL

## 5. 推荐的数据格式

建议不要只靠 `run.log` 事后解析，而是直接在任务管理器里主动写结构化数据。

推荐每个 episode 一个目录：

```text
episode_xxxx/
  scene.json
  config.json
  result.json
  stage_trace.jsonl
  candidates.jsonl
  trajectories.npz
  joint_feedback.npz
```

各文件建议：

- `scene.json`
  - 柜体、目标盒、clutter、target pose
- `config.json`
  - 算法参数、模型版本、planner 配置
- `result.json`
  - 是否成功、最终阶段、总耗时、失败原因
- `stage_trace.jsonl`
  - 每个阶段的输入、输出、状态切换
- `candidates.jsonl`
  - 所有 `pre-insert` 候选及其标签
- `trajectories.npz`
  - 规划轨迹张量
- `joint_feedback.npz`
  - 执行期实际回读

## 6. 现在立刻最该补的记录

如果只补最小闭环，我建议先补这 6 项：

1. `SELECT_PRE_INSERT` 生成的全部候选及标签
2. 每个阶段最终选用的 group 和 planner id
3. 每条成功轨迹的完整 `JointTrajectory`
4. 每条失败轨迹的失败类别
5. 执行前起始 joint state 与执行后最终 joint state
6. `base` 与 `r1` 的阶段运动量

这 6 项一补，马上就能开始做：

- 候选排序
- 成功率预测
- 参数回归

## 7. 这些数据怎么用

### 第一阶段

先做离线监督学习：

- 目标不是替代 MoveIt
- 而是给 MoveIt 提供更好的候选、参数和策略选择

也就是让模型负责：

- 选哪个 `pre-insert`
- 用哪个 group
- `base_standoff` 设多少

而把最终轨迹生成仍交给现有 MoveIt。

### 第二阶段

当你积累到足够多的成功和失败 episode 后，可以再做：

- 学一个更强的候选打分器
- 学一个插入阶段的 fallback 策略
- 学一个轨迹先验，用来给 IK / planner 提供 seed

### 第三阶段

只有当你已经稳定记录了：

- 场景
- 初始状态
- 候选
- 完整规划轨迹
- 实际执行反馈

才值得考虑：

- 离线 RL
- diffusion trajectory policy
- 端到端动作生成

## 8. 对当前项目的建议结论

结论很明确：

- 当前成功数据有价值
- 但最适合训练“规划前的决策模型”
- 不适合直接训练“整套动作策略模型”

所以最现实的路线是：

1. 继续随机实验积累成功/失败 case
2. 补结构化候选与轨迹记录
3. 先做 `pre-insert` 候选排序和参数回归
4. 再考虑更重的学习方法
