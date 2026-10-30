# 变质量火箭轨道规划与预测控制

二维变质量火箭的两段燃烧入轨规划，以及含大气相关扰动的 MPC 轨迹跟踪。当前实验比较 **纯 PSO、实数编码 GA、多起点 SA**，全部取消 SQP 等局部精修。

## 当前实验

目标高度 600 km。每种算法运行 5 个配对种子，每次 96 个候选、180 轮，共 17,376 次轨迹评价；同组初始点、控制边界、动力学及规划扰动完全一致。SA 是 96 条独立退火链，不能将它表述为单链 SA。

| 算法 | MATLAB 高精度复核通过数 | 复核目标中位数 |
|---|---:|---:|
| PSO | 5/5 | 1.749 × 10⁻⁶ |
| GA | 0/5 | 2.295 × 10⁻⁴ |
| 多起点 SA | 0/5 | 5.218 × 10⁻³ |

这是本算例、预算及具体算子设置下的比较，不是算法一般优越性或真实飞行成功率的证明。候选复核只重算固定控制，不再优化参数。完整逐次数据位于 [trials.csv](results/comparison/trials.csv)。

纯 PSO 参考的独立扰动实验中，MPC 位置跟踪 RMS 为 1.945 m，开环为 25,836.9 m；259 次 QP 均成功。跟踪误差相对于参考，入轨残差相对于目标圆轨道，二者分开报告。

## 阅读与目录

- [论文 PDF](paper/main.pdf) / [LaTeX 工程说明](paper/README.md)：当前主文与实验分析。
- [实验复现说明](experiments/README.md)：环境、预算、种子、输入指纹和复现命令。
- config/：物理、轨道、控制和可视化配置。
- model/：共用动力学、扰动与 MATLAB ODE 积分。
- optimization/：纯 PSO、GA、SA 及共用参数化、评价器。
- experiments/：论文主实验的 Python/Numba 搜索与 MATLAB 独立复核。
- control/：LTV-MPC、线性化与凸 QP。
- validation/：物理校验、算法预算/复现检查。
- results/comparison/：本次无精修实验原始数据、元数据与 CSV。
- results/data/：历史数据和当前纯 PSO 参考。
- visualization/、poster/、演示文档/：动画、历史海报与演示材料。

论文当前数值只取 results/comparison/。历史文件中的 PSO+SQP、紧约束与放宽约束结果不参与当前对比；旧海报与旧说明保留其历史含义。历史版论文可从 Git 中本次更新之前的版本检索。

## 运行

MATLAB 中先运行 startup，也可使用统一入口：

~~~matlab
run_rocket('optimize')          % 纯 PSO，成功后验证轨道
run_rocket('optimize-ga')       % 真正的实数编码 GA
run_rocket('optimize-sa')       % 多起点 SA
run_rocket('mpc')               % 当前纯 PSO 参考的闭环跟踪
run_rocket('animate')           % MPC / 开环双火箭彗星动画
run_rocket('compare-planners')  % 全 MATLAB 版等预算对比（不是论文随机流）
~~~

独立 MATLAB 入口与论文的 Python 搜索采用不同随机数实现，不要求逐位一致。**复现论文数值请使用 experiments/README.md 中的主流程。** 该主流程不覆盖历史精修结果，默认复用兼容的缓存；要生成全新的随机试验，应另行设定种子并保持三方法配对协议。

主实验使用 Python 3.13、Numba 16 线程与 MATLAB R2025b。MPC 的 quadprog 需要 Optimization Toolbox；MATLAB GPU/parfor 路径需要 Parallel Computing Toolbox。自编三搜索器不依赖 Global Optimization Toolbox。图表和论文需要 Matplotlib、TeX Live / XeLaTeX。

正常运行优化、MPC 或动画入口会覆盖相应默认输出。存档的本次主实验及论文复现路径独立保存。

## 数学模型与限制

中心引力、二次阻力、指数大气、固定有效排气速度、密度相关有限时间相关高斯激励。地球不自转、火箭为平面质点，未建模分级、姿态执行机构、测量噪声、动压/热流/过载约束。教学参数不对应实际型号，不能直接用于工程发射设计。

论文算法目标是终端圆轨道误差，不包含燃料最小项。MPC 沿参考线性化，在理想全状态反馈下滚动求解凸 QP；未证明递归可行性或严格闭环稳定性。
