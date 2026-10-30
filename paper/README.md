# 学术论文工程

主文件 main.tex，输出 main.pdf，A4 中文论文，使用 XeLaTeX 编译两次。当前正文由模型、数值方法、无精修规划器比较、LTV-MPC、结果、讨论、结论及可复现性附录组成。

本次修订取消课程作业式页眉、口语化叙述和大量代码/控制节点清单。所有正文结果来自 results/comparison/ 的纯 PSO / GA / 多起点 SA 实验与重新运行的纯 PSO 参考 MPC，不沿用历史 PSO+SQP 的终端精度或闭环统计。

~~~powershell
# 在项目根目录生成图表和数字
python experiments/report_comparison.py
# 随后在 paper/ 目录编译
xelatex -interaction=nonstopmode -halt-on-error main.tex
xelatex -interaction=nonstopmode -halt-on-error main.tex
~~~

- sections/：分节正文。
- comparison_values.tex：由数据生成的数值宏。
- comparison_table.tex、mpc_table.tex：由数据生成的表格。
- figures/fig_planner_comparison.pdf：五次搜索曲线和复核目标。
- figures/fig_planner_residuals.pdf：全部复核分量残差。
- figures/fig_pure_reference.pdf：无精修 PSO 参考。
- figures/fig_pure_mpc.pdf：基于该参考的开环/闭环及 QP 计时。
- figures/comparison_summary.json：数字与数据来源清单。

历史 figures/ 中的旧图、旧 summary.json 和 make_figures.py 为可追溯性保留，不参与当前正文。复现实验应遵循 experiments/README.md。原 26 页课程式论文可从 Git 本次更新之前的提交检索。
