# 论文：变推力—变质量火箭的两段燃烧入轨规划与模型预测控制

本目录是完整论文（26 页，A4，XeLaTeX + ctex），正文数据基于**放宽边界后的解**
（偏角 ±15°、滑行 ≤400 s），并在 §6.7 与紧约束旧解作对比。

- `main.pdf`：编译产物。
- `main.tex`：主文件（版式、标题、摘要、章节装配）。
- `sections/`：分节源文件
  - `01_intro` 引言；`02_model` 数学物理模型；`03_numerics` 数值求解与验证；
    `04_planning` 两段燃烧规划；`05_mpc` LTV-MPC 与 QP；`06_results` 结果与分析
    （含 §6.7 边界敏感性）；`07_discussion` 讨论；`08_conclusion` 结论；
    `appendix` 符号表与复现；`refs` 参考文献。
- `figures/`：11 幅矢量图（PDF）与生成脚本
  - `make_figures.py`：只读取 `../results/data/*.mat`，用 matplotlib 重绘全部图，
    同时把文中引用的关键数值写入 `summary.json`。
  - `summary.json`：论文所有数值的机读快照（规划残差、MPC 指标、QP 统计等）。

## 编译

在本目录执行（需要 TeX Live，含 ctex 与 Fandol 字体）：

```powershell
xelatex -interaction=nonstopmode main.tex
xelatex -interaction=nonstopmode main.tex   # 第二遍生成书签与交叉引用
```

或用 TeXstudio 打开 `main.tex`，编译器选 XeLaTeX，编译两次。
`main.tex` 首行已声明 `% !TeX program = xelatex`，TeXstudio 会自动识别。

## 重新生成图

```powershell
python figures/make_figures.py
```

要求 `numpy`、`scipy`、`matplotlib`。脚本不修改任何 MATLAB 结果，只读取
`results/data/` 下的 `rocket_two_burn_relaxed.mat`、`rocket_mpc_relaxed.mat`、
`rocket_orbit_coast_relaxed.mat`、`rocket_acceleration_benchmark.mat`，
以及用于对比的 `rocket_two_burn_result.mat`（紧约束旧解）。

## 数据来源

| 文件 | 用途 |
|---|---|
| `results/data/rocket_two_burn_relaxed.mat` | 正文规划结果（放宽边界） |
| `results/data/rocket_mpc_relaxed.mat` | 闭环与开环对照、QP 统计（放宽边界） |
| `results/data/rocket_orbit_coast_relaxed.mat` | 关机后一个周期的滑行验证 |
| `results/data/rocket_two_burn_result.mat` | 紧约束旧解，仅用于 §6.7 对比 |
| `results/data/rocket_acceleration_benchmark.mat` | GPU 与串行 CPU 批量计时 |

生成这些文件的三步计算在项目根的 `experiments/` 中：

```powershell
matlab -batch "run('experiments/relaxed_bounds.m')"   # 放宽边界的 PSO + SQP
matlab -batch "run('experiments/relaxed_verify.m')"   # 滑行验证 + MPC
python experiments/compare_relaxed.py                 # 对比表与图
```

复现说明与随机种子见论文附录 B。
