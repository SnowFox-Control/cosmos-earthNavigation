# 火箭上升、两段燃烧入轨与极坐标动画

在 MATLAB 中打开本目录，使用统一入口：

```matlab
run_rocket('optimize')   % 两段燃烧PSO + 可选SQP，成功后验证轨道
run_rocket('mpc')        % PSO参考轨迹的MPC闭环跟踪
run_rocket('benchmark')  % GPU与串行CPU批量计时
run_rocket('animate')    % MPC与开环双火箭同步彗星动画
run_rocket('verify')     % 物理一致性检查与已保存轨道验证
run_rocket('report')     % 重画已保存的优化结果图
run_rocket('demo')       % 原始24吨火箭示例
```

也可先运行 `startup`，再直接调用函数。所有结果通过 `rocket_file` 定位，不依赖工作目录。

| 目录 | 内容 |
|---|---|
| config/ | 物理、轨道、优化、动画参数 |
| model/ | 共享非线性动力学、极坐标变质量ODE |
| control/ | 时变线性化MPC、约束QP与闭环仿真 |
| optimization/ | 两段燃烧PSO/SQP和之前的连续燃烧方案 |
| visualization/ | 动画与结果图 |
| validation/ | 物理一致性、入轨滑行验证 |
| examples/ | 原始24吨火箭示例 |
| results/data/ | MAT数据、CSV控制节点 |
| results/figures/ | 静态图和动画关键帧 |
| results/animations/ | GIF、MP4 |
| results/checkpoints/ | PSO中间粒子群 |
| docs/ | 参数指南、模型说明、结果记录 |

常用入口：

- [火箭与轨道](config/rocket_orbit_config.m)：质量、推进剂占比、比冲、目标高度、验收容差。
- [两段燃烧优化](config/rocket_two_burn_config.m)：粒子数、迭代数、随机种子、控制范围、局部修正开关。
- [动画设置](config/rocket_visual_config.m)：圈数、播放速度、缩放、颜色、输出格式。
- [基础模型](config/rocket_config.m)：引力、大气、阻力、面积、初始状态、原始示例控制。

详见 [参数调整指南](docs/参数调整指南.md)。
**物理或优化参数改动后重跑 optimize；仅改动画设置直接运行 animate。**
动画使用保存结果中的实际参数，不会将新配置强行套到旧轨迹。

当前结果：

- [两段燃烧结果说明](docs/两段燃烧优化结果.md)
- [极坐标视频](results/animations/rocket_polar_comet.mp4)
- [控制节点CSV](results/data/rocket_orbit_controls.csv)
- `results/data/rocket_two_burn_result.mat`：最终完整结果。
- `results/data/rocket_two_burn_ga_stage.mat`：历史纯GA阶段结果（保留）。
- `results/data/rocket_two_burn_pso_stage.mat`：下次优化产生的纯PSO阶段结果。

优化默认需要 Global Optimization Toolbox 和 Optimization Toolbox；动画使用 Image Processing Toolbox。
运行会覆盖同名输出，比较方案时请先另存MAT/CSV/动画。
早期连续燃烧程序仍保留，推荐日常使用上述统一入口。

当前默认优化器为 `particleswarm`，主程序为 `optimization/rocket_two_burn_pso.m`。旧函数名 `rocket_two_burn_ga` 是转调PSO的兼容入口。历史连续燃烧GA程序和历史结果仍保留，不由默认入口调用。

动力学现已支持密度比例高斯加速度扰动，默认开启。配置位于 `config/rocket_config.m` 的 `p.noise`，验证函数为 `verify_disturbance`。已有结果保持原参数，新扰动需重新计算。

GPU默认自动启用RTX 5060的CUDA前向兼容，粒子群使用GPU批量RK4，最终仍由ode45/SQP复核。GPU不可用时回退4进程CPU并行。MPC参数在 `config/rocket_mpc_config.m`。详见 [GPU加速与MPC闭环控制](docs/GPU加速与MPC闭环控制.md)。


当前默认动画为白底、无渐变的双火箭comet比较：`run_rocket('animate')`。
蓝色圆点为MPC火箭，红色三角为开环火箭，两者共享时间轴、初始条件和实际扰动。
配置在 `config/rocket_comparison_visual_config.m`，MP4/GIF输出为 `results/animations/rocket_mpc_comparison.*`。
默认保留关机后一圈无推力飞行；设置 `orbitPeriods=0` 可只播放上升段。
原单参考轨迹的渐变动画改用 `run_rocket('animate-reference')`。

双火箭动画现同时显示两枚火箭的实时无推力二体密切轨道（淡虚线）及目标圆轨道（绿色点划线），运行 `run_rocket('animate')` 重新生成。
