"""Expand the retained A4 sources into A3 portrait posters."""
from pathlib import Path
P = Path(__file__).resolve().parent
style = (P/'poster_style.tex').read_text(encoding='utf-8')
style = style.replace('a4paper,landscape,margin=8mm', 'a3paper,portrait,margin=11mm')
style = style.replace(r'\fontsize{11}{13}', r'\fontsize{13.5}{16}')
style = style.replace(r'\fontsize{7.1}{8.3}', r'\fontsize{8.6}{10.7}')
style = style.replace(r'\fontsize{6.7}{8}', r'\fontsize{8}{10}')
style = style.replace(r'\fontsize{20}{22}', r'\fontsize{24}{27}')
style = style.replace(r'\fontsize{10}{12}', r'\fontsize{11}{14}')
style = style.replace(r'\vspace{3mm}\hspace{3mm}', r'\vspace{5mm}\hspace{3mm}')
style = style.replace(r'\end{minipage}\vspace{3mm}', r'\end{minipage}\vspace{5mm}')
style += '\n'+r'\newcommand{\extrablock}[1]{\par\vspace{5pt}{\color{teal}\bfseries\fontsize{12}{15}\selectfont #1}\par\vspace{2pt}}'+'\n'
(P/'poster_a3_style.tex').write_text(style,encoding='utf-8')

extra_model_zh = r'''
\extrablock{物理参数与能力估算}
{\fontsize{9.2}{12}\selectfont
\begin{tabular*}{\linewidth}{@{\extracolsep{\fill}}lr@{}}\toprule
初始质量 / 干质量 & $60\,000 / 1\,200$ kg\\
有效排气速度 & $4\,500$ m/s\\
阻力系数 / 迎风面积 & $0.35 / 2.0$ m$^2$\\
海平面密度 / 标高 & $1.225$ kg/m$^3$ / $8.5$ km\\
参考密度下扰动标准差 & $0.1$ m/s$^2$\\\bottomrule
\end{tabular*}}
目标圆轨道速度约 $7.562$ km/s，能量给出的理想速度增量下界约 $8.243$ km/s。实际消耗后的质量比约 $9.636$，高于必要下界；差异来自有限推力、转向和外力损失。
\extrablock{数值积分：控制到轨迹的桥梁}
对 $\dot y=f(t,y)$，经典 RK4 使用四次斜率评价：
\[
 y_{n+1}=y_n+\frac{\Delta t}{6}(K_1+2K_2+2K_3+K_4).
\]
GPU 批量 RK4 加速候选评估，自适应 Dormand--Prince 方法复核最终轨迹。燃尽与落地作为事件处理；步长细化时固定同一扰动样本，避免把随机差异误认为积分误差。
\takeaway{理想方程给出能力边界；含外力的 ODE 才决定实际轨迹。}
'''
extra_model_en = r'''
\extrablock{Physical settings and capability}
{\fontsize{9.2}{12}\selectfont
\begin{tabular*}{\linewidth}{@{\extracolsep{\fill}}lr@{}}\toprule
Initial / dry mass & $60\,000 / 1\,200$ kg\\
Effective exhaust speed & $4\,500$ m/s\\
Drag coefficient / area & $0.35 / 2.0$ m$^2$\\
Sea-level density & $1.225$ kg/m$^3$\\
Density scale height & $8.5$ km\\
Disturbance at reference density & $0.1$ m/s$^2$\\\bottomrule
\end{tabular*}}
Circular speed is about $7.562$ km/s; the energy-based ideal velocity bound is $8.243$ km/s. The achieved mass ratio is $9.636$, above the necessary bound, reflecting finite thrust, steering and external-force losses.
\extrablock{Numerical integration links control to flight}
For $\dot y=f(t,y)$, classical RK4 evaluates four slopes:
\[
 y_{n+1}=y_n+\frac{\Delta t}{6}(K_1+2K_2+2K_3+K_4).
\]
Batched GPU RK4 accelerates candidate evaluation; adaptive Dormand--Prince integration checks the final trajectory. Fuel depletion and impact are events. Step refinement uses the same forcing realization to separate integration error from random variation.
\takeaway{The ideal equation bounds capability; the ODE with external forces determines the flight.}
'''
pso_zh = r'''
\subhead{粒子群如何搜索？}
在归一化设计空间中，粒子位置为 $\xi_i$、搜索速度为 $\nu_i$；个体最佳与邻域最佳分别记为 $p_i,g_i$：
\[
\begin{aligned}
 \nu_i^{k+1}&=\omega\nu_i^k+c_1r_1\odot(p_i-\xi_i^k)\\
 &\quad+c_2r_2\odot(g_i-\xi_i^k),\\
 \xi_i^{k+1}&=\xi_i^k+\nu_i^{k+1}.
\end{aligned}
\]
随机探索与群体经验共同降低终端目标；越界、落地及推进剂超限候选受到限制或惩罚。PSO 不保证全局最优，本研究也不以最小燃料为目标。
'''
pso_en = r'''
\subhead{How the swarm searches}
In normalized design space, particle position and search velocity are $\xi_i,\nu_i$; personal and neighborhood bests are $p_i,g_i$:
\[
\begin{aligned}
 \nu_i^{k+1}&=\omega\nu_i^k+c_1r_1\odot(p_i-\xi_i^k)\\
 &\quad+c_2r_2\odot(g_i-\xi_i^k),\\
 \xi_i^{k+1}&=\xi_i^k+\nu_i^{k+1}.
\end{aligned}
\]
Random exploration and shared experience reduce terminal error. Bounds, impact and propellant-budget violations are constrained or penalized. PSO does not guarantee a global optimum; fuel minimization is not the objective here.
'''
cost_zh = r'''
\subhead{滚动时域代价与约束}
\[
\begin{aligned}
 J_{\rm MPC}={}&\sum_{j=1}^{N}\|e_{k+j|k}\|_{Q_j}^2
 +\sum_{j=0}^{N-1}\|v_{k+j|k}\|_R^2\\
 &+\sum_{j=0}^{N-1}\|\Delta v_{k+j|k}\|_{R_\Delta}^2
 +\lambda\|\epsilon\|^2.
\end{aligned}
\]
$e$ 与 $v$ 已按物理尺度归一化。误差权重、控制幅值与变化惩罚兼顾精度和平滑性；地面高度用软约束，干质量及输入边界采用硬约束。
'''
cost_en = r'''
\subhead{Receding-horizon cost and constraints}
\[
\begin{aligned}
 J_{\rm MPC}={}&\sum_{j=1}^{N}\|e_{k+j|k}\|_{Q_j}^2
 +\sum_{j=0}^{N-1}\|v_{k+j|k}\|_R^2\\
 &+\sum_{j=0}^{N-1}\|\Delta v_{k+j|k}\|_{R_\Delta}^2
 +\lambda\|\epsilon\|^2.
\end{aligned}
\]
States and controls are scaled. Tracking, effort and change penalties balance accuracy with smoothness. Ground clearance is softened; dry-mass and input limits are hard constraints.
'''
kkt_zh = r'''
\subhead{为什么是凸 QP？}
状态消元后，$H=2(\mathcal B^\mathsf T\bar Q\mathcal B+\bar R+D^\mathsf T\bar R_\Delta D)$；正定输入权重使其正定。凸内点法求解扰动 KKT 条件，并逐步减小障碍参数，逼近约束最优解。
\smallnote{使用沿参考轨迹的时变线性模型（LTV-MPC），而非完整 NMPC。未来扰动取零均值，控制器不读取实际未来噪声。}
'''
kkt_en = r'''
\subhead{Why is the QP convex?}
After state elimination, $H=2(\mathcal B^\mathsf T\bar Q\mathcal B+\bar R+D^\mathsf T\bar R_\Delta D)$. Positive-definite input weighting makes $H$ positive definite. The interior-point solver follows perturbed KKT conditions as the barrier parameter decreases.
\smallnote{This is reference-based LTV-MPC, not full NMPC. Future disturbances are assigned zero mean; the controller cannot read future plant noise.}
'''
for lang in ['zh','en']:
    s=(P/f'poster_{lang}.tex').read_text(encoding='utf-8')
    s=s.replace('poster_style.tex','poster_a3_style.tex')
    s=s.replace(r'\fontsize{8.5}{10.5}',r'\fontsize{10}{13}').replace(r'\fontsize{8.7}{10.3}',r'\fontsize{10.3}{12.7}')
    s=s.replace(r'\fontsize{8.1}{10.3}',r'\fontsize{9.2}{12}')
    s=s.replace('height=31mm','height=42mm').replace('height=36mm','height=48mm').replace('height=30mm','height=40mm')
    s=s.replace(r'\columnend\colsep', (extra_model_zh if lang=='zh' else extra_model_en)+'\n'+r'\columnend\colsep',1)
    marker=r'\includegraphics[width=\linewidth,height=42mm]'
    s=s.replace(marker,(pso_zh if lang=='zh' else pso_en)+'\n'+marker,1)
    start=s.index(r'\subhead{ODE 前向求解}' if lang=='zh' else r'\subhead{Forward integration}')
    end=s.index(r'\subhead{MPC:',start) if lang=='en' else s.index(r'\subhead{MPC：',start)
    s=s[:start]+s[end:]
    marker='预测误差、控制幅值、变化率' if lang=='zh' else 'Quadratic penalties on predicted tracking errors'
    s=s.replace(marker,(cost_zh if lang=='zh' else cost_en)+'\n'+marker,1)
    marker=r'\takeaway{离线规划' if lang=='zh' else r'\takeaway{Offline planning'
    s=s.replace(marker,(kkt_zh if lang=='zh' else kkt_en)+'\n'+marker,1)
    marker=r'\includegraphics[width=\linewidth,height=48mm]'
    s=s.replace(marker,r'\includegraphics[width=\linewidth,height=60mm]{images/reference_'+lang+'.pdf}\n'+marker,1).replace('.pdf}\\n\\includegraphics','.pdf}\n\\includegraphics')
    marker=r'\includegraphics[width=\linewidth,height=40mm]{images/error_'+lang+'.pdf}'
    s=s.replace(marker,marker+'\n'+r'\includegraphics[width=\linewidth,height=40mm]{images/velocity_'+lang+'.pdf}',1)
    marker=r'\takeaway{反馈显著' if lang=='zh' else r'\takeaway{Feedback suppresses'
    note_zh=r'''\extrablock{阶段解释与在线求解}
第一段建立上升与横向速度，滑行抬升高度，第二段提高切向速度并压低径向速度。参考关机高度误差为 $0.265$ m，径向与切向速度残差为 $-0.0212$、$0.0463$ m/s。
MPC 每 2 s 更新，预测 30 s；本次 230 次 QP 均成功，平均求解约 1.793 ms（不含参考线性化预计算）。计时只代表本机仿真，不等于嵌入式实时性保证。
'''
    note_en=r'''\extrablock{Flight phases and online computation}
The first burn builds ascent and transverse motion; coasting gains altitude; the second burn raises transverse speed and reduces radial speed. Reference cutoff residuals are $0.265$ m, $-0.0212$ m/s and $0.0463$ m/s.
MPC updates every 2 s with a 30 s horizon. All 230 QPs succeeded; mean solve time was 1.793 ms, excluding reference linearization. Desktop timing does not establish embedded real-time capability.
'''
    s=s.replace(marker,(note_zh if lang=='zh' else note_en)+'\n'+marker,1)
    (P/f'poster_a3_{lang}.tex').write_text(s,encoding='utf-8')
print('Created A3 portrait source files; A4 originals retained.')

