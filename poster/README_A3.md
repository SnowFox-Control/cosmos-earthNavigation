# A3 竖向三栏学术 Poster

中英文各一页，纸张为 A3 竖向，297 × 420 mm。原 A4 横向版本保留。

- `poster_a3_zh.tex` / `poster_a3_zh.pdf`：中文版主文件与 PDF。
- `poster_a3_en.tex` / `poster_a3_en.pdf`：英文版主文件与 PDF。
- `poster_a3_style.tex`：共用的 A3 版式。
- `images/`：中文及英文 PDF 矢量图。
- `make_a3.py`：由保留的 A4 源文件生成 A3 源文件；直接修改 A3 源文件后，无需运行此脚本。

使用 TeXstudio 选择 XeLaTeX，分别打开两个 A3 主文件编译；不需要 BibTeX。

```powershell
xelatex -interaction=nonstopmode -halt-on-error poster_a3_zh.tex
xelatex -interaction=nonstopmode -halt-on-error poster_a3_en.tex
```

打印请选择 A3、竖向、实际大小（100%）。中文使用 TeX Live 自带 Fandol 字体。

相较 A4 版，补充了物理参数、理想能力估算、简述 RK4、PSO 迭代公式、MPC 二次代价与约束、QP 凸性、参考高度/速度、速度跟踪误差、燃烧阶段解释及在线求解统计。

数据沿用论文冻结快照，不重新运行优化。目标高度为 600 km；开闭环使用相同实际扰动，该扰动与规划样本不同。结果保留单次试验与简化模型的适用边界。
