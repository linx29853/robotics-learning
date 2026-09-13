# Robotics Learning｜机器人学学习记录

从平面二连杆机械臂出发，记录对 **正运动学 → 逆运动学 → 雅可比 → 奇异性 → MuJoCo 可视化** 的学习过程。

## 学习进度与证据

| 阶段 | 学习内容 | 验证方式 | 入口 |
| --- | --- | --- | --- |
| 01 已完成基础实验 | 标准 DH、齐次变换、几何正解 | 100 组 DH / 几何对比 | [正运动学](experiments/forward_kinematics.py) |
| 02 已完成基础实验 | 余弦定理、双解、可达性 | FK 回代、内外边界、不可达点 | [逆运动学](experiments/inverse_kinematics.py) |
| 03 正在巩固 | 雅可比列向量与关节速度贡献 | 本次整理补充 100 组中心差分 | [雅可比](experiments/jacobian.py) |
| 04 正在学习 | 速度椭圆、奇异性 | 102 个姿态与 MuJoCo 对比 | [MuJoCo 演示](simulation/jacobian_demo.py) |

学习经历见 [学习日志](notes/learning-log.md)，详细推导见 [正逆运动学笔记](notes/正逆运动学学习笔记.md)。笔记使用 Markdown 和 LaTeX，可由 Typora 阅读。

## 模型约定

平面 2R 机械臂：a1=0.30 m、a2=0.25 m。q1 相对世界 +X 轴，q2 相对第一连杆；内部角度使用弧度，角速度使用 rad/s，末端速度使用 m/s。不考虑碰撞和实际关节限位。

$$
x=a_1\cos q_1+a_2\cos(q_1+q_2),\quad y=a_1\sin q_1+a_2\sin(q_1+q_2)
$$

$$
\dot p=J(q)\dot q,\qquad \det J=a_1a_2\sin q_2
$$

位置可达范围为 0.05 ≤ r ≤ 0.55 m；伸直和折叠处雅可比退化。逆解针对位置任务，通常有两个分支，边界分支会重合。等长杆目标为原点时有无穷多解，当前函数只提供代表解。

## 目录

```text
experiments/   整理后的可运行实验和验证
simulation/   MuJoCo 二连杆交互演示
notes/        公式笔记、学习日志和下一步
archive/      原始练习快照，保留学习痕迹
```

`archive` 保留原始练习；整理版修正脚本入口、统一参数，并增强验证。MuJoCo 演示与文档整理使用 AI 辅助，目标是帮助我理解和复做，不能将辅助生成的验证等同于本人已独立掌握。此次提交是已有学习资料的归档，不重造此前提交时间线。

## 快速开始

已验证环境：Windows、Python 3.13.14、NumPy 2.5.3、MuJoCo 3.13.0。

```powershell
git clone https://github.com/linx29853/robotics-learning.git
cd robotics-learning
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe experiments/forward_kinematics.py
.\.venv\Scripts\python.exe experiments/inverse_kinematics.py
.\.venv\Scripts\python.exe experiments/jacobian.py
.\.venv\Scripts\python.exe simulation/jacobian_demo.py --check
.\.venv\Scripts\python.exe simulation/jacobian_demo.py
```

如果只做前三个数学实验，只需安装 NumPy。

在仓库根目录运行：

```powershell
& 'E:\robot-learning\mujoco-env\Scripts\python.exe' simulation/jacobian_demo.py
```

## MuJoCo 怎么看

| 按键 | 操作与观察 |
| --- | --- |
| 1 | 弯曲，红蓝箭头不共线，绿色曲线为椭圆 |
| 2 | 伸直，两列共线，椭圆退化为线段 |
| 3 | 折叠，两箭头反向共线，同样奇异 |
| 4 | 接近伸直，椭圆很窄 |
| A / D | 第一关节减 / 加 3° |
| S / W | 第二关节减 / 加 3° |
| T | 连续转动第二关节 / 停止 |

点击画面后使用按键。红箭头对应 J 第一列，蓝箭头对应第二列；绿色曲线将单位关节速度圆经 J 映射得到。所有速度按 0.4 秒显示比例叠放于末端。绿色曲线是速度边界，不是末端轨迹或位置工作空间。

演示直接设置 qpos 并调用 mj_forward。它展示运动学，不模拟力矩控制或动态跟踪。终端显示 det(J) 与最小奇异值，帮助理解接近奇异时某些方向的速度能力下降。

## 我的几个关键收获

1. NumPy 的 `@` 才是矩阵乘法，DH 连乘不能用 `*`。
2. 逆解必须 FK 回代，不能只观察角度数值。
3. 检查并裁剪 cos(q2) 后才能开平方。
4. 测试不可达点时要明确验证异常，不能残留使用上一轮变量。
5. J 每一列代表一个关节的瞬时速度贡献；奇异是独立末端速度方向减少，关节仍然可以运动。

## 来源与后续

学习起点：[rparak/2-Link_Manipulator](https://github.com/rparak/2-Link_Manipulator)。此前复现记录：[2-Link-Manipulator-Learning](https://github.com/linx29853/2-Link-Manipulator-Learning)。本仓库只整理学习代码和笔记，不包含上游完整工程、模型素材或动画资源。

参考：[MuJoCo 官方 Python 文档](https://mujoco.readthedocs.io/en/stable/python.html)。

下一步：有限差分复做 → 数值逆运动学 → 阻尼伪逆与奇异处理 → 轨迹规划 → 关节控制。
