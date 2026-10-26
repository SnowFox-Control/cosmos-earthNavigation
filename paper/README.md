# 论文：变推力—变质量火箭的两段燃烧入轨规划与模型预测控制

本目录是课程研究的完整论文（24 页，A4，XeLaTeX + ctex）。

- `main.pdf`：编译产物。
- `main.tex`：主文件（版式、标题、摘要、章节装配）。
- `sections/`：分节源文件
  - `01_intro` 引言；`02_model` 数学物理模型；`03_numerics` 数值求解与验证；
    `04_planning` 两段燃烧规划；`05_mpc` LTV-MPC 与 QP；`06_results` 结果与分析；
    `07_discussion` 讨论；`08_conclusion` 结论；`appendix` 符号表与复现；`refs` 参考文献。
- `figures/`：10 幅矢量图（PDF）与生成脚本
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
`results/data/` 下的 `rocket_two_burn_result.mat`、`rocket_mpc_result.mat`、
`rocket_orbit_coast.mat`、`rocket_acceleration_benchmark.mat`。

## 数据来源

论文中的全部数值来自项目已有的保存结果（2026-10-20 冻结快照，目标 600 km）：

| 文件 | 用途 |
|---|---|
| `results/data/rocket_two_burn_result.mat` | 规划结果（设计变量、终端残差、时序、质量） |
| `results/data/rocket_mpc_result.mat` | 闭环与开环对照、QP 统计 |
| `results/data/rocket_orbit_coast.mat` | 关机后一个轨道周期的滑行验证 |
| `results/data/rocket_acceleration_benchmark.mat` | GPU 与串行 CPU 批量计时 |

复现计算见论文附录 B，或在项目根目录运行 `run_rocket('optimize')`、
`run_rocket('mpc')`、`run_rocket('benchmark')`、`run_rocket('verify')`。
