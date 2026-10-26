# 火箭轨迹跟踪：MPC 数学模型、线性化与 QP 推导

本文对照本项目当前实现，解释“火箭动力学怎样一步步变成一个能够在线求解的二次规划问题”。先建立直观认识，再给出必要推导。所有公式统一使用单个美元符号对包围。

## 1. MPC 到底在控制什么？

PSO 轨道规划提供了参考状态轨迹和参考控制。可以把它们理解为一张带时间的飞行计划：每个时刻应该在哪里、速度是多少，以及发动机应该怎样工作。

开环控制只照着计划执行。遇到扰动后，即使控制指令仍然正确，实际状态也可能逐渐偏离计划。MPC 则反复做四件事：

1. 读取当前火箭状态，判断已经偏离了多少。
2. 用动力学模型预测未来一小段时间的运动。
3. 寻找一组满足约束的控制修正，让未来误差尽量小。
4. **只执行这组修正的第一项**，下个采样时刻重新预测、重新优化。

“预测一段、执行一步、再算一遍”就是滚动时域控制。它不需要一次算出直到任务结束的全部反馈策略。

本项目采用时变线性模型预测控制（LTV-MPC）。火箭实际运动仍由非线性方程计算；为了让在线优化更快，控制器把参考轨迹附近的运动近似成线性模型，再求解凸二次规划（QP）。

## 2. 状态、控制与非线性模型

### 2.1 五个状态、两个控制修正

跟踪状态为

$x=\begin{bmatrix}r&\theta&u&w&m\end{bmatrix}^{\mathsf T}\in\mathbb R^5.$

| 符号              | 含义      | 单位  |
| --------------- | ------- | --- |
| $r$             | 地心距离    | m   |
| $\theta$        | 极角      | rad |
| $u=\dot r$      | 径向速度    | m/s |
| $w=r\dot\theta$ | 切向速度    | m/s |
| $m$             | 火箭当前总质量 | kg  |

累计冲量可以作为第六个积分状态，但不进入当前 MPC 的跟踪状态。恒定有效排气速度下，冲量已经与推进剂消耗存在确定关系。

参考输入为 $\delta_{\mathrm{ref}}(t)$ 和 $q_{\mathrm{ref}}(t)$，分别表示推力相对速度方向的偏角和推进剂排出流率。MPC 优化的是它们的修正量：

$c_k=\begin{bmatrix}\Delta\delta_k\\\Delta q_k\end{bmatrix}.$

在一个控制周期 $t\in[t_k,t_{k+1})$ 内，施加

$\delta(t)=\delta_{\mathrm{ref}}(t)+\Delta\delta_k,\qquad q(t)=q_{\mathrm{ref}}(t)+\Delta q_k.$

**保持不变的是反馈修正，不是参考前馈。** 即使同一个周期内参考流率正在变化，MPC 仍可以保持一个常量修正。

### 2.2 非线性火箭方程

记 $V=\sqrt{u^2+w^2}$、$\beta=\operatorname{atan2}(w,u)$、$\alpha=\beta+\delta$，推力为 $T=c_e q$。这里用 $c_e$ 表示有效排气速度，避免与控制修正向量 $c_k$ 混淆。

空气密度与阻力系数项为

$\rho(r)=\rho_0\exp[-\max(r-R_E,0)/H_a],\qquad K(r)=\frac12\rho(r)C_DA_f.$

其中 $H_a$ 为大气标高，$A_f$ 为迎风面积。动力学为

$\begin{aligned}\dot r&=u,\\\dot\theta&=w/r,\\\dot u&=\frac{w^2}{r}+\frac{c_eq\cos\alpha}{m}-\frac{\mu}{r^2}-\frac{K(r)Vu}{m}+a_{\varepsilon,r},\\\dot w&=-\frac{uw}{r}+\frac{c_eq\sin\alpha}{m}-\frac{K(r)Vw}{m}+a_{\varepsilon,\theta},\\\dot m&=-q.\end{aligned}$

这是非线性方程，因为包含状态相乘、除法、三角函数和大气密度的指数变化。起飞初期程序短暂保持竖直推力方向；速度接近零时也对方向角进行保护。

实际飞行中的高斯扰动幅值正比于当地大气密度。**MPC 预测采用未来扰动为零的名义模型**，而不是提前知道未来噪声。在线修正来自每次重新读取状态，不能把这种做法等同于严格的随机 MPC 或鲁棒 MPC。

## 3. 从连续方程得到“一步预测”

连续模型写成 $\dot x=f(t,x,c)$。优化器需要回答的是：给定当前状态和本周期修正，经过一个采样周期后，状态变成什么？

因此定义离散映射

$x_{k+1}=F_k(x_k,c_k).$

本项目中的 $F_k$ 是用 RK4 对名义动力学积分一个控制周期得到的映射。若一个积分子步为 $h$，则

$\begin{aligned}K_1&=f(t,x,c),\\K_2&=f(t+h/2,x+hK_1/2,c),\\K_3&=f(t+h/2,x+hK_2/2,c),\\K_4&=f(t+h,x+hK_3,c),\\x^+&=x+\frac h6(K_1+2K_2+2K_3+K_4).\end{aligned}$

每次评价 $f$ 时，参考控制取对应时刻的值，修正 $c$ 保持不变。重复若干子步，就得到 $F_k$。

**连续动力学描述物理规律；离散映射描述一个采样间隔内的预测。** 后面的线性化针对整个离散映射进行。

## 4. 线性化：用参考附近的“局部斜率”代替复杂曲面

### 4.1 从泰勒展开到误差方程

一元函数在 $a$ 附近满足 $g(a+\Delta x)\approx g(a)+g'(a)\Delta x$。多变量情况一样，只是导数变成 Jacobian 矩阵。

在参考点 $(x_k^{\mathrm{ref}},0)$ 附近展开：

$F_k(x_k^{\mathrm{ref}}+\Delta x_k,c_k)\approx F_k(x_k^{\mathrm{ref}},0)+A_k^x\Delta x_k+B_k^c c_k,$

其中

$A_k^x=\left.\frac{\partial F_k}{\partial x}\right|_{(x_k^{\mathrm{ref}},0)},\qquad B_k^c=\left.\frac{\partial F_k}{\partial c}\right|_{(x_k^{\mathrm{ref}},0)}.$

$A_k^x$ 回答“当前状态多偏一点，下一时刻会多偏多少”；$B_k^c$ 回答“当前修正多加一点，下一时刻会改变多少”。矩阵维度分别为 $5\times5$ 和 $5\times2$。

减去下一时刻的参考状态，得到

$\Delta x_{k+1}=A_k^x\Delta x_k+B_k^c c_k+d_k^x,$

$d_k^x=F_k(x_k^{\mathrm{ref}},0)-x_{k+1}^{\mathrm{ref}}.$

### 4.2 为什么不能随手丢掉 $d_k^x$？

如果参考精确满足同一个离散映射，$d_k^x=0$。但实际参考来自另一套高精度积分和插值，规划还可能使用了具体扰动样本，而 MPC 使用无未来扰动的名义预测。

所以，即使当前完全处于参考状态，名义预测的下一步也不一定恰好落在参考上。$d_k^x$ 就保留了这部分已知偏差。

它是**参考与名义离散模型之间的仿射偏差**，不是对未来实际随机扰动的预知。

### 4.3 Jacobian 在程序里怎样得到？

当前程序对离散映射使用中心差分。例如，第 $\ell$ 个状态方向的导数列近似为

$A_k^x(:,\ell)\approx\frac{F_k(x_k^{\mathrm{ref}}+\epsilon_\ell b_\ell,0)-F_k(x_k^{\mathrm{ref}}-\epsilon_\ell b_\ell,0)}{2\epsilon_\ell}.$

其中 $b_\ell$ 为第 $\ell$ 个坐标单位向量。对两个控制修正方向也分别施加正负微扰，可得到 $B_k^c$。

这相当于让模型进行两次几乎相同的试飞：一次略微增加某个量，一次略微减少，然后比较终点差异。

足够光滑时，中心差分具有二阶截断精度；启停、饱和、方向角保护附近可能不光滑，导数只代表局部近似。

### 4.4 一个连续动力学导数例子

为了理解导数的物理含义，可以暂看连续模型的输入 Jacobian。非竖直保持阶段、未触及饱和时，在固定状态下：

$B_{\mathrm{cont}}=\frac{\partial f}{\partial(\delta,q)}=\begin{bmatrix}0&0\\0&0\\-\frac{c_eq\sin\alpha}{m}&\frac{c_e\cos\alpha}{m}\\\frac{c_eq\cos\alpha}{m}&\frac{c_e\sin\alpha}{m}\\0&-1\end{bmatrix}.$

第一列说明偏角改变推力在径向和切向的分配；第二列说明增加流率会增大推力，也会更快消耗质量。滑行时 $q=0$，偏角的一阶推进作用自然消失。

状态导数还必须考虑速度方向变化。例如 $V>0$ 时，$\partial\beta/\partial u=-w/V^2$，因此

$\frac{\partial\dot u}{\partial u}=\frac{c_eqw\sin\alpha}{mV^2}-\frac{K}{m}\left(V+\frac{u^2}{V}\right).$

可见，对速度求导时不能把推力方向角当成常数。这个连续 Jacobian 仅用于解释；**当前实现直接差分整个 RK4 离散映射，并不是使用 $A_d=I+T_sA_{\mathrm{cont}}$ 的一阶欧拉近似。**

## 5. 归一化：避免数值量级支配优化

地心距离以百万米计，角度可能很小，质量又以千克计。直接把这些误差平方相加，会使权重难以解释，也可能恶化数值条件。

定义正对角尺度矩阵

$S_x=\operatorname{diag}(s_r,s_\theta/R_E,s_u,s_w,s_m),\qquad S_c=\operatorname{diag}(s_\delta,s_q),$

并令

$e_k=S_x^{-1}(x_k-x_k^{\mathrm{ref}}),\qquad v_k=S_c^{-1}c_k.$

这里 $s_\theta$ 是用米表示的地表弧长误差尺度，因此极角尺度要除以 $R_E$。例如，径向误差为 $50$ m、尺度为 $500$ m，则归一化误差为 $0.1$。这只是换单位，不改变真实物理过程。

代入上一节方程，得到实际使用的模型：

$\boxed{e_{k+1}=A_ke_k+B_kv_k+d_k}$，

$A_k=S_x^{-1}A_k^xS_x,\qquad B_k=S_x^{-1}B_k^cS_c,\qquad d_k=S_x^{-1}d_k^x.$

后面的 $e,v$ 都是归一化量。求解后通过 $c_k=S_cv_k$ 换回实际控制修正。

## 6. 把未来多步预测展开成一个矩阵

固定当前时刻 $k$，用 $e_0$ 表示当前误差，$e_j=e_{k+j|k}$ 表示“在当前时刻预测的第 $j$ 步误差”。本节的 $A_j,B_j,d_j$ 对应从当前开始的未来第 $j$ 个区间。

前两步为

$e_1=A_0e_0+B_0v_0+d_0,$

$e_2=A_1A_0e_0+A_1B_0v_0+B_1v_1+A_1d_0+d_1.$

第一步控制既影响第一步，也影响第二步；第二步控制只影响第二步及以后。这种“过去影响未来，未来不能改变过去”的结构，会形成块下三角矩阵。

预测 $N$ 步，将未来状态与控制堆叠：

$E=\begin{bmatrix}e_1\\e_2\\\vdots\\e_N\end{bmatrix}\in\mathbb R^{5N},\qquad U=\begin{bmatrix}v_0\\v_1\\\vdots\\v_{N-1}\end{bmatrix}\in\mathbb R^{2N}.$

统一写成

$\boxed{E=\mathcal A e_0+\mathcal B U+\mathcal D=a+\mathcal B U}$。

其中 $a=\mathcal A e_0+\mathcal D$ 为不施加未来修正时的预测误差。

以 $N=3$ 为例：

$\mathcal A=\begin{bmatrix}A_0\\A_1A_0\\A_2A_1A_0\end{bmatrix},\qquad\mathcal B=\begin{bmatrix}B_0&0&0\\A_1B_0&B_1&0\\A_2A_1B_0&A_2B_1&B_2\end{bmatrix},$

$\mathcal D=\begin{bmatrix}d_0\\A_1d_0+d_1\\A_2A_1d_0+A_2d_1+d_2\end{bmatrix}.$

这一步叫作**状态消元或凝聚**：将未来状态全部表达成未来控制的函数，优化器不必同时把每个状态都当作独立未知数。

## 7. 从“希望飞得好”写出二次代价

MPC 兼顾跟踪精度、修正幅度、修正平滑性和软约束违背程度：

$\begin{aligned}J={}&\sum_{j=1}^{N}e_j^{\mathsf T}Q_je_j+\sum_{j=0}^{N-1}v_j^{\mathsf T}Rv_j\\&+\sum_{j=0}^{N-1}(v_j-v_{j-1})^{\mathsf T}R_\Delta(v_j-v_{j-1})+\lambda_\epsilon\epsilon^{\mathsf T}\epsilon.\end{aligned}$

| 部分    | 作用              |
| ----- | --------------- |
| 状态误差项 | 希望位置、速度和质量接近参考  |
| 控制幅值项 | 避免反馈修正过大        |
| 控制变化项 | 避免反馈修正相邻周期剧烈跳动  |
| 松弛惩罚项 | 尽量不违反软化后的地面高度限制 |

采用 $Q_j\succeq0$、$R\succ0$、$R_\Delta\succeq0$、$\lambda_\epsilon>0$。当前实现还放大最后一步状态权重；这是终端加权，并不是终端不变集约束。

这里惩罚的是**反馈修正的相邻变化**。参考控制自身也随时间变化，所以它不等于对实际总控制变化率施加硬限制。

### 7.1 用差分矩阵表示控制变化

令 $v_{-1}$ 为上个周期已经执行的归一化修正。定义

$D=\begin{bmatrix}I_2&0&0&\cdots\\-I_2&I_2&0&\cdots\\0&-I_2&I_2&\cdots\\\vdots&\ddots&\ddots&\ddots\end{bmatrix},\qquad b_\Delta=\begin{bmatrix}v_{-1}\\0\\\vdots\\0\end{bmatrix}.$

$D$ 为 $2N\times2N$ 块双对角矩阵。于是 $DU-b_\Delta$ 的各块恰好为 $v_0-v_{-1},v_1-v_0,\ldots$。

再定义

$\bar Q=\operatorname{blkdiag}(Q_1,\ldots,Q_N),\quad\bar R=I_N\otimes R,\quad\bar R_\Delta=I_N\otimes R_\Delta.$

代价就可以写成

$J=E^{\mathsf T}\bar Q E+U^{\mathsf T}\bar R U+(DU-b_\Delta)^{\mathsf T}\bar R_\Delta(DU-b_\Delta)+\lambda_\epsilon\epsilon^{\mathsf T}\epsilon.$

## 8. 展开代价，得到 QP 的 H 和 f

将 $E=a+\mathcal B U$ 代入。跟踪项展开为

$\begin{aligned}(a+\mathcal B U)^{\mathsf T}\bar Q(a+\mathcal B U)={}&U^{\mathsf T}\mathcal B^{\mathsf T}\bar Q\mathcal B U\\&+2U^{\mathsf T}\mathcal B^{\mathsf T}\bar Q a+a^{\mathsf T}\bar Q a.\end{aligned}$

控制变化项展开为

$\begin{aligned}(DU-b_\Delta)^{\mathsf T}\bar R_\Delta(DU-b_\Delta)={}&U^{\mathsf T}D^{\mathsf T}\bar R_\Delta DU\\&-2U^{\mathsf T}D^{\mathsf T}\bar R_\Delta b_\Delta+b_\Delta^{\mathsf T}\bar R_\Delta b_\Delta.\end{aligned}$

两式的最后一项与 $U$ 无关，可以从优化目标中删除：它们改变目标数值，却不改变最优控制。

按标准 QP 的 $\frac12U^{\mathsf T}H_UU+f_U^{\mathsf T}U$ 整理：

$\boxed{H_U=2(\mathcal B^{\mathsf T}\bar Q\mathcal B+\bar R+D^{\mathsf T}\bar R_\Delta D)}$，

$\boxed{f_U=2(\mathcal B^{\mathsf T}\bar Q a-D^{\mathsf T}\bar R_\Delta b_\Delta)}.$

系数 $2$ 来自 QP 标准形式自带的 $\frac12$，不是额外增加了一次惩罚。

将松弛并入未知量 $\zeta=\begin{bmatrix}U\\\epsilon\end{bmatrix}$，得到

$H=\begin{bmatrix}H_U&0\\0&2\lambda_\epsilon I_N\end{bmatrix},\qquad f=\begin{bmatrix}f_U\\0\end{bmatrix}.$

程序还对 $H$ 做极小的对角正则化及数值对称化，减小舍入误差的影响。

### 8.1 为什么这是凸问题？

对任意非零向量 $z$：

$\frac12z^{\mathsf T}H_Uz=(\mathcal Bz)^{\mathsf T}\bar Q(\mathcal Bz)+z^{\mathsf T}\bar Rz+(Dz)^{\mathsf T}\bar R_\Delta(Dz)>0.$

前后两项非负，中间项因 $\bar R\succ0$ 严格为正，所以 $H_U\succ0$。加上正定松弛惩罚后，目标仍严格凸。

只要约束可行，该 QP 就有唯一最优解。但**局部近似的 QP 是凸的，不代表原始非线性火箭轨迹优化已成为全局凸问题。**

## 9. 将物理约束转成线性不等式

### 9.1 偏角、流率与修正幅度

物理输入满足

$-\delta_{\max}\le\delta_{\mathrm{ref}}(t)+\Delta\delta\le\delta_{\max},\qquad0\le q_{\mathrm{ref}}(t)+\Delta q\le q_{\max}.$

修正本身也有幅度边界。若一个周期内修正保持常数，为保证整个周期都有效，需要

$\Delta\delta\ge-\delta_{\max}-\min_t\delta_{\mathrm{ref}}(t),\qquad\Delta\delta\le\delta_{\max}-\max_t\delta_{\mathrm{ref}}(t),$

$\Delta q\ge-\min_tq_{\mathrm{ref}}(t),\qquad\Delta q\le q_{\max}-\max_tq_{\mathrm{ref}}(t).$

这里的极值只在当前控制周期内取。程序检查周期端点与内部控制插值节点，再与最大修正幅度求交，除以控制尺度后堆叠为 $\underline U\le U\le\overline U$。

保留原燃烧时序时，滑行区间与最终关机后的修正固定为零；跨越滑行边界的某些周期也保守地禁用修正。当前 MPC 不额外搜索新的点火时机。

### 9.2 地面高度软约束

令 $C_r$ 从堆叠误差 $E$ 中选出每一步径向误差，定义归一化参考高度

$g_j=(r_{k+j}^{\mathrm{ref}}-R_E)/s_r.$

原始地面约束是 $g+C_rE\ge0$。为降低近地阶段局部线性误差导致 QP 不可行的风险，引入非负松弛：

$g+C_rE+\epsilon\ge0,\qquad\epsilon\ge0.$

代入预测式，整理得到

$-C_r\mathcal B U-\epsilon\le g+C_ra.$

如果 $\epsilon_j=0.01$、径向尺度为 $500$ m，意味着该线性预测允许约 $5$ m 的高度约束放宽。它不代表真实火箭可以穿过地面：非线性飞行对象仍设置落地终止事件。

### 9.3 干质量硬约束

令 $C_m$ 选出每一步归一化质量误差，定义归一化参考燃料余量

$b_{m,j}=(m_{k+j}^{\mathrm{ref}}-m_{\mathrm{dry}})/s_m.$

预测质量不能低于干质量，即 $b_m+C_mE\ge0$。因此

$-C_m\mathcal B U\le b_m+C_ma.$

这里没有质量松弛，属于硬约束。

### 9.4 完整 QP

$\boxed{\min_{\zeta}\ \frac12\zeta^{\mathsf T}H\zeta+f^{\mathsf T}\zeta}$，

满足 $G\zeta\le b$，以及

$\begin{bmatrix}\underline U\\0\end{bmatrix}\le\zeta\le\begin{bmatrix}\overline U\\+\infty\end{bmatrix},$

其中

$G=\begin{bmatrix}-C_r\mathcal B&-I_N\\-C_m\mathcal B&0\end{bmatrix},\qquad b=\begin{bmatrix}g+C_ra\\b_m+C_ma\end{bmatrix}.$

本项目预测 $N=15$ 步，每步两个控制，因此 $U\in\mathbb R^{30}$；每步一个高度松弛，最终 $\zeta\in\mathbb R^{45}$。未来状态已经消元，所以不必向 QP 再单独提交动力学等式约束。

## 10. QP 怎么求解：凸内点法

当前程序调用 MATLAB quadprog，明确选择 interior-point-convex。标准目标与线性约束的接口形式见 [MathWorks 官方 quadprog 文档](https://www.mathworks.com/help/optim/ug/quadprog.html)。下面解释内点法的数学思路，不逐行复现求解器内部实现。

将上下界也并入 $\widetilde G\zeta\le\widetilde b$。引入约束余量 $s=\widetilde b-\widetilde G\zeta\ge0$ 和乘子 $\lambda\ge0$，最优解满足 KKT 条件：

$\begin{aligned}H\zeta+f+\widetilde G^{\mathsf T}\lambda&=0,\\\widetilde G\zeta+s-\widetilde b&=0,\\s_i\lambda_i&=0,\qquad s_i,\lambda_i\ge0.\end{aligned}$

最后一条叫互补条件：如果一条约束还很宽松，乘子应为零；如果约束正好卡住解，其乘子可以为正。

内点法先将互补条件换成 $s_i\lambda_i=\tau$，其中 $\tau>0$。它在内部沿一系列近似解前进，再逐步减小 $\tau$。

令 $S=\operatorname{diag}(s)$、$\Lambda=\operatorname{diag}(\lambda)$，定义残差

$r_d=H\zeta+f+\widetilde G^{\mathsf T}\lambda,\quad r_p=\widetilde G\zeta+s-\widetilde b,\quad r_c=S\lambda-\tau\mathbf1.$

对这些方程做牛顿线性化，方向满足

$\begin{bmatrix}H&\widetilde G^{\mathsf T}&0\\\widetilde G&0&I\\0&S&\Lambda\end{bmatrix}\begin{bmatrix}\Delta\zeta\\\Delta\lambda\\\Delta s\end{bmatrix}=-\begin{bmatrix}r_d\\r_p\\r_c\end{bmatrix}.$

求出方向后，选择合适步长更新，使 $s,\lambda$ 保持正值。不断减小残差和障碍参数，解就逼近原 QP 的约束最优解。

通俗地说：**QP 规定误差与控制代价怎样权衡、哪些边界不能越过；内点法负责在这些边界之间找到代价最小的位置。**

软化地面约束不保证整个 QP 永远可行，因为输入边界和干质量等硬约束仍可能冲突。

## 11. 一个可以手算的两步例子

暂时忽略五维火箭状态，只考虑 $e_{j+1}=e_j+v_j$，当前 $e_0=2$，预测两步，代价为 $J=e_1^2+e_2^2+v_0^2+v_1^2$。本例不包含控制变化惩罚和松弛，仅说明凝聚与 QP。

$E=\begin{bmatrix}e_1\\e_2\end{bmatrix}=\begin{bmatrix}2\\2\end{bmatrix}+\begin{bmatrix}1&0\\1&1\end{bmatrix}\begin{bmatrix}v_0\\v_1\end{bmatrix}.$

因此

$H=2\left(\begin{bmatrix}1&1\\0&1\end{bmatrix}\begin{bmatrix}1&0\\1&1\end{bmatrix}+I\right)=\begin{bmatrix}6&2\\2&4\end{bmatrix},$

$f=2\begin{bmatrix}1&1\\0&1\end{bmatrix}\begin{bmatrix}2\\2\end{bmatrix}=\begin{bmatrix}8\\4\end{bmatrix}.$

无约束时解 $HU+f=0$，得到 $U^*=(-1.2,-0.4)^{\mathsf T}$。若每步修正必须位于 $[-1,1]$，第一项就超界了。

有约束的最优解为 $U^*=(-1,-0.5)^{\mathsf T}$，对应预测误差 $e_1=1$、$e_2=0.5$。可以验证：在第一项卡住下界 $v_0=-1$ 后，对第二项求导得到 $v_1=-0.5$。

MPC 此时只执行 $v_0=-1$。下一次读到真实状态后重新建立问题，并不承诺照搬之前预测的第二项 $-0.5$。扰动出现后，这个“重新看一眼再决定”的过程就是反馈发挥作用的地方。

## 12. 一次实际控制周期的完整过程

1. 读取真实状态，计算当前归一化误差 $e_0$。
2. 取出未来各段的 $A_j,B_j,d_j$ 与输入边界。当前实现沿参考预先计算这些模型，不是在每次测得的实际状态处重新线性化。
3. 展开预测，得到 $E=a+\mathcal B U$。
4. 用状态误差、控制幅值和变化惩罚构造 $H,f$，并加入高度松弛。
5. 生成地面、干质量和输入约束，求解凸 QP。
6. 取前两个分量 $v_0^*$，换回 $c_k=S_cv_0^*$，叠加到参考控制并施加给非线性对象。
7. 下个周期重新读取状态，重复以上步骤。

QP 未成功时，当前实现退回零反馈修正，即仅执行参考前馈；命令仍经过物理限幅及燃烧时序处理。这是明确的回退策略，不构成对所有异常状态的安全性证明。

方法的适用范围由局部线性模型决定：离参考太远时，QP 即使成功，也可能给出不够准确的真实运动预测。当前还假设完整状态可获得，未加入状态估计、执行机构动态、终端不变集或递归可行性证明。

## 13. 数学符号与项目实现对照

| 数学内容                       | 对应文件或变量                                   |
| -------------------------- | ----------------------------------------- |
| 非线性动力学 $f$                 | model/rocket_dynamics.m                   |
| 连续参考状态和输入                  | control/rocket_mpc_reference.m            |
| 离散映射 $F_k$                 | control/rocket_mpc_step.m                 |
| 归一化 Jacobian $A_k,B_k,d_k$ | control/rocket_mpc_linearize.m            |
| 预测矩阵 $\mathcal B$、自由预测 $a$ | control/rocket_mpc_qp.m 的 prediction、free |
| 差分矩阵 $D$、上步修正项 $b_\Delta$  | 同文件的 D、prior                              |
| QP 的 $H,f,G,b$             | 同文件的 H、f、Aineq、bineq                      |
| 控制叠加、限幅和时序                 | control/rocket_mpc_command.m              |
| 滚动控制与开环对照                  | control/rocket_mpc_run.m                  |

推导的核心连接是：**非线性积分得到一步映射，泰勒展开得到局部模型，递推消元将未来状态写成未来控制的函数，二次代价与线性约束整理后便得到 QP。**
