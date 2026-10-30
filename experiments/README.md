# 无精修规划器对比：复现说明

## 实验协议

- 方法：标准全局最佳 PSO、实数编码 GA、96 条独立链的多起点 SA。
- 初始点：每组种子三方法逐元素相同；50% 均匀随机，50% 两段燃烧启发式。
- 优化种子：161、162、163、164、165；各方法后续随机流由种子及方法名称确定。
- 预算：96 × (180 + 1) = 17,376 次评价，包含初始批次与 GA 精英重评。
- 控制：10 维两段燃烧参数，偏角 ±15°，滑行 20–400 s；全部使用相同边界。
- 目标：归一化关机高度、径向速度、切向速度误差平方和；燃料超限与触地罚项一致。
- 规划激励种子：20261020；实际 MPC 对象激励种子：20261021。
- 搜索积分：双精度 RK4，最大步长 0.05 s；不是白噪声 SDE。
- 精修：**全部禁用**。MATLAB 对返回的固定控制做两档 ode45 复核，不修改参数。
- 验收：高度误差 ≤500 m，两速度误差分别 ≤10 m/s，预算内且不触地。
- 主结果：五次中位数 / IQR / 全部散点 / 通过数；五次不是足够大的统计鲁棒性样本。

## 环境与安装

参考运行：Windows、Python 3.13、NumPy 2.3.5、SciPy 1.17.0、Matplotlib 3.10.8、Numba 0.68.1、LLVM 0.50.0、MATLAB R2025b。Python 环境依赖已固定在 requirements.txt。

~~~powershell
python -m venv .venv
.venv/Scripts/Activate.ps1
python -m pip install -r experiments/requirements.txt
~~~

MPC 需要 MATLAB Optimization Toolbox。正式搜索使用多核 CPU，不要求 NVIDIA GPU。可选 CUDA 后端依赖适合本机 CUDA 的 CuPy；RTX 5060 的较新架构可能需要 PTX 前向兼容。CUDA 内核编译时应使用 ASCII 临时目录。不要把不同后端的评价耗时混入算法排序。

## 主流程（项目根目录）

已有输入与完整结果已随仓库提供。仅重画图、表和编译论文，无须再次优化：

~~~powershell
python experiments/report_comparison.py
Set-Location paper
xelatex -interaction=nonstopmode -halt-on-error main.tex
xelatex -interaction=nonstopmode -halt-on-error main.tex
~~~

完整重算流程如下。相同数字内容的输入有相同指纹；缓存检查不受 MAT 文件头时间戳影响。

~~~powershell
matlab -batch "run('startup.m'); export_planner_inputs();"
python experiments/compare_planners.py --backend numba
matlab -batch "run('startup.m'); verify_comparison();"
python experiments/report_comparison.py
~~~

搜索脚本默认复用相同协议及输入指纹的缓存。要检验搜索本身的可复现性，可先将 results/comparison/ 下已有 *_pure.json 移到另一个明确命名的备份目录，再运行搜索；不要移走 inputs.mat。不要将不同物理参数、边界或预算的结果放入同一归档后静默复用，指纹不符会中止。

受限环境下如果 MATLAB 预设目录不可写，可以在命令前将 MATLAB_PREFDIR 指向项目 tmp/ 下的独立目录；tmp/ 不入库。

## 文件含义

| 文件 | 内容 |
|---|---|
| inputs.mat | 从 MATLAB 导出的物理配置、五组初始点、精确高斯样本 |
| *_pure.json | 每方法/种子的搜索最优参数、全历史、评价数、耗时和输入指纹 |
| campaign.json | 正式 15 次搜索的总清单 |
| pure_candidates.mat | 提交 MATLAB 复核的固定控制参数 |
| *_verified.mat | 每次高精度积分、原控制及细化差值 |
| verification.json | 15 次高精度验收结果 |
| best_pso_reference.mat | 通过验收的 PSO 中复核目标最小的参考，未精修 |
| mpc_pure_pso.mat / mpc_metrics.json | 该纯 PSO 参考的重新控制实验 |
| trials.csv | 全部逐次指标，避免仅公布最好结果 |
| summary.json | 汇总与源文件 SHA256 |
| cuda_probe.mat / numba_probe.mat | 相同 96 个候选的后端一致性检查 |
| backend_probe.json | 一次试算的计时及后端误差，不是全面硬件基准 |

GA 使用三元锦标赛、扩展算术混合交叉、每坐标 1/10 概率的高斯变异、一个精英。SA 每轮每链扰动一个坐标，温度 1 → 1e-6，提议宽度 0.2 → 0.005，链间不交换。当前配置没有对每方法进行等额调参搜索，因此不能把结果解释为最优调参后的范式排名。

Numba/CUDA 快路径明确断言当前 600 km、1 m 初始高度、零初速、5 s 竖直保持、1 s 激励间隔和0.5 s启停过渡。改变这些结构设置时，需同步扩展快路径；断言会阻止不兼容设置被静默忽略。

## 检查

~~~powershell
python -m unittest discover -s validation -p test_planner_search.py
matlab -batch "run('startup.m'); verify_pso();"
~~~

检查覆盖评价预算、相同初始点、随机可复现性、边界修复、历史最优单调性、CPU/CUDA 存档一致性、共用解码器及实际 MATLAB 动力学冒烟测试。CPU/CUDA 存档测试不需要实时 GPU。

## 历史与替代入口

compare_planners.m 提供全 MATLAB 的自编纯算法对比，随机数流及评估后端与论文主流程不同，结果单独保存到 results/comparison_matlab/，不覆盖或混合论文主实验。历史 relaxed_bounds.m、relaxed_verify.m、compare_relaxed.py 对应旧的边界敏感性工作流；不要用其历史输出替代当前纯算法实验。当前默认不再进行局部精修。

make_figures.py（paper/figures/ 下）仍是历史精修数据的作图脚本；本论文的新图只由 report_comparison.py 生成。
