# 中英文学术 Poster

两版均为横向 A4（297 × 210 mm）、单页、三栏；图表为 PDF 矢量图。

- `poster_zh.tex` / `poster_zh.pdf`：中文版。
- `poster_en.tex` / `poster_en.pdf`：英文版。
- `poster_style.tex`：两版共用的字体、颜色和版式。
- `images/`：分别使用中文、英文标签的矢量图。
- `main.tex`：默认编译中文版；可直接打开各语言主文件。

在本目录运行，或在 TeXstudio 中分别打开两个主文件，选择 XeLaTeX：

```powershell
xelatex -interaction=nonstopmode -halt-on-error poster_zh.tex
xelatex -interaction=nonstopmode -halt-on-error poster_en.tex
```

无需 BibTeX。中文采用 TeX Live 提供的 Fandol 字体。打印时选择 A4 横向、实际大小（100%）。

内容由上一版论文压缩而来：保留理想能力与质量比界限、二维变质量动力学、PSO/SQP 两段燃烧规划、MPC 凸 QP 与开闭环对照；压缩 ODE 数值求解细节，省略代码与超参数。数值和图表沿用上级 `data/experiment.npz` 的 2026-10-20 冻结快照，目标高度为 600 km，没有重新优化。
