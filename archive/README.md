# 原始学习快照

这里保留从旧学习项目复制的四份练习，未改写代码，用于对照学习过程。
它们是整理时的快照，不表示完整 Git 历史，也不是伪造的阶段提交。

- `01_forward_kinematics.py`：DH 与解析公式、随机验证。存在 main 保护之外依赖局部初始化变量的问题，不建议作为模块导入。
- `forward_kinematics_simple_exp.py`：早期几何正运动学练习。
- `inverse_kinematics.py`：可达性、双解和边界测试。
- `jacobian.py`：关节速度映射及两列贡献。

推荐执行 `../experiments/` 内的整理版。整理版统一参数、入口和断言，并补充有限差分验证。
