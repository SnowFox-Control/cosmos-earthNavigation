# GPU加速与PSO参考轨迹的MPC闭环控制

## 1. 已完成的实现

当前默认优化采用“GPU批量PSO评估 + CPU高精度ODE复核 + 可选并行SQP”。GPU不可用时回退到CPU并行评估。
控制部分以已经成功入轨的PSO轨迹作为完整状态参考，采用时变线性化、约束二次规划和滚动时域反馈。

2026年10月20日本机实际验证：MATLAB R2025b，NVIDIA GeForce RTX 5060 Laptop GPU，计算能力12.0，显存约8 GB。
开启 `parallel.gpu.enableCUDAForwardCompatibility(true)` 后可正常调用CUDA；无需迁移到Python。

## 2. GPU与并行加速

### 2.1 GPU批量轨迹

`optimization/rocket_gpu_fitness.m` 把一批候选控制送入 `gpuArray/arrayfun`。每个CUDA线程独立推进一条火箭轨迹，计算对应终端误差及落地/燃料惩罚。
标量GPU内核使用与原模型相同的引力、二次阻力、变质量、两段燃烧控制和密度比例高斯扰动，但积分算法为固定步长RK4，默认最大步长0.05 s。
随机样本先由独立固定种子生成再上传GPU，保持不同候选之间相同的随机环境。

GPU批处理用于粒子群的候选筛选。最终候选、SQP目标评估和最终验收仍由CPU `ode45` 完成，不能以GPU阶段适应度直接宣称入轨成功。
固定步长近似与自适应ODE的结果会有数值差异；接近终端精度要求时应依赖高精度复核或局部修正。

### 2.2 CPU并行与配置

在 `config/rocket_two_burn_config.m` 设置：

| 设置 | 默认值 | 含义 |
|---|---|---|
| `accelerationBackend` | `'auto'` | 自动尝试GPU；也可用 `'gpu'` 或 `'cpu'` |
| `gpuStep` | 0.05 | GPU RK4最大步长，s |
| `useParallel` | true | 允许CPU进程池 |
| `parallelWorkers` | 4 | 新建进程池时的工作进程数 |
| `parallelLocalRefinement` | true | SQP中央差分并行评估 |

GPU PSO使用 `UseVectorized=true`，不同时把同一个GPU任务分发到多个CPU进程。
CPU回退使用 `particleswarm.UseParallel=true`；MPC离线线性化预计算也可使用 `parfor`。
进程池在同一MATLAB会话内复用，首次启动和CUDA首次编译有额外开销。
若GPU初始化或内核编译失败，程序给出具体警告并回退CPU；实际后端保存在结果的 `gpuInfo/parallelInfo` 中。

### 2.3 本机批量计时

对同一条已有控制重复组成96条轨迹、在GPU预热后测得：

| 计算方式 | 批次时间 |
|---|---:|
| 串行CPU `ode45` | 9.834 s |
| RTX 5060批量RK4 | 0.324 s |

该次比值约30.4，但这是**GPU固定步长批处理与CPU自适应ODE的综合差异**，不是相同算法相同误差下的纯硬件加速比。
该测试的归一化目标值相对差约2.03%；重复轨迹测试也不能代表所有种群的负载分布。
计时结果保存在 `results/data/rocket_acceleration_benchmark.mat`，可运行 `run_rocket('benchmark')` 重新测量。

## 3. MPC状态与参考

参考必须来自 `result.optimizer='PSO'` 且 `result.success=true` 的结果文件。
跟踪状态为

$\boldsymbol x=[r,\theta,u,w,m]^{\mathsf T}$。

发动机冲量仍在实际仿真中积分记录，但因其与质量消耗有确定关系，不作为独立跟踪状态。
参考状态及前馈控制从保存的PSO解插值得到。预测时域超过参考关机时刻时，使用无推力、无随机扰动的同一动力学继续外推参考，而不是冻结火箭位置。

误差按

$\widetilde{\boldsymbol e}=\operatorname{diag}(s_r,s_\theta/R_E,s_u,s_w,s_m)^{-1}(\boldsymbol x-\boldsymbol x_{\mathrm{ref}})$

归一化。极角乘地球半径后形成地表弧长误差，以避免弧度和米混合造成病态权重。
默认尺度为 `[500 m, 500 m, 30 m/s, 30 m/s, 1000 kg]`。

## 4. 预测模型与滚动优化

`model/rocket_dynamics.m` 是规划器、MPC预测及实际飞行共享的非线性方程。
每个参考采样点通过RK4离散映射及中央差分得到局部时变模型：

$\widetilde{\boldsymbol e}_{k+1}=A_k\widetilde{\boldsymbol e}_k+B_k\boldsymbol v_k+\boldsymbol d_k$。

其中控制是对PSO前馈的修正：

$\delta=\delta_{\mathrm{ref}}+\Delta\delta,\qquad q=q_{\mathrm{ref}}+\Delta q$。

归一化修正为 $\boldsymbol v=[\Delta\delta/(2^\circ),\Delta q/(50\ \mathrm{kg/s})]^{\mathsf T}$。
仿射项 $\boldsymbol d_k$ 计入参考插值与名义离散模型之间的差异。预测采用高斯扰动零均值，不读取实际飞行的未来随机样本。
这是LTV-MPC，不是每个时刻求解完整非线性最优控制问题的NMPC。

默认控制周期2 s、预测15步，即30 s。在线目标为

$J=\sum_{j=1}^{N}\widetilde{\boldsymbol e}_{k+j}^{\mathsf T}Q_j\widetilde{\boldsymbol e}_{k+j}+\sum_{j=0}^{N-1}\boldsymbol v_{k+j}^{\mathsf T}R\boldsymbol v_{k+j}+\sum_{j=0}^{N-1}(\boldsymbol v_{k+j}-\boldsymbol v_{k+j-1})^{\mathsf T}R_\Delta(\boldsymbol v_{k+j}-\boldsymbol v_{k+j-1})+\lambda\|\boldsymbol s\|^2$。

终端状态权重默认放大5倍。约束包含推力偏角/流率物理边界、修正幅值、干质量下界和带松弛变量的地面高度约束。
默认最大修正为5度、100 kg/s；修正变化有代价惩罚，但没有额外声称已施加执行机构摆角速率硬限制。
默认保留两段燃烧的滑行和关机时序，滑行期间不新增推力。

QP通过 Optimization Toolbox 的 `quadprog` 求解，不要求使用MPC Toolbox的控制器对象。
每次只执行第一步修正；下一控制周期读取新状态重新求解，参考前馈控制在周期内继续连续变化。
起飞保持段后才启用反馈，默认在首个不早于5 s的控制采样点，即6 s启用。

若QP失败，明确记录失败标志并回退到参考前馈；实际对象仍有燃尽/落地事件保护。
已有理想状态测量直接用于反馈，尚未加入传感器误差、状态估计、通讯延迟或执行机构动力学。
由MATLAB桌面计时不能据此认定已满足真实嵌入式实时性要求。

## 5. 含扰动实际对象与对照试验

实际对象用共享非线性方程和高精度 `ode45` 推进，并加入密度比例高斯扰动。
默认实际飞行种子20261021、径向/切向参考密度标准差均为0.1 m/s²；它与规划器随机环境不同。
开环和MPC采用完全相同的实际初始状态、扰动种子及模型，开环只执行PSO前馈，构成公平的反馈对照。

2026年10月20日使用已保存的600 km成功PSO轨迹，全上升段465.509 s的结果：

| 指标 | 开环 | MPC |
|---|---:|---:|
| 位置跟踪RMS（径向与地表弧长合成） | 64147 m | 2.65 m |
| 径向/切向速度跟踪RMS合成 | 317.07 m/s | 0.097 m/s |
| 关机高度目标误差 | -88214 m | 0.544 m |
| 关机径向速度 | -577.40 m/s | -0.0057 m/s |
| 关机切向速度目标误差 | 237.83 m/s | 0.0027 m/s |

230次QP调用均成功；单次平均约1.9 ms、最大约83 ms，时间不包括离线线性化预计算。
这些是当前模型、当前参考轨迹、一个独立扰动种子及理想状态反馈条件下的仿真结果，不是多种子成功率或全局稳定性证明。
结果保存在 `results/data/rocket_mpc_result.mat`，图为 `results/figures/rocket_mpc_tracking.png`。

## 6. 使用入口与调整

```matlab
run_rocket('optimize')   % GPU PSO + CPU ode45/SQP，生成新参考
run_rocket('mpc')        % 加载已有成功PSO参考，执行闭环与开环对照
run_rocket('benchmark')  % GPU/串行CPU计时与目标值差异
```

MPC配置在 `config/rocket_mpc_config.m`：

| 配置 | 含义 |
|---|---|
| `sampleTime`, `predictionSteps` | 控制周期与预测步数 |
| `startTime` | 起飞后启用反馈的最早时间 |
| `stateScale`, `stateWeights` | 状态误差尺度和权重 |
| `inputWeights`, `inputRateWeights` | 修正幅值及变化代价 |
| `maxCorrection` | 偏角rad、流率kg/s最大修正 |
| `plantNoiseSeed`, `plantNoiseSigma` | 实际飞行扰动环境 |
| `initialOffset` | 初始r、theta、u、w、m偏差 |
| `preserveBurnSchedule` | 是否禁止在预定滑行段施加推力 |
| `useParallel`, `parallelWorkers` | 离线预测模型并行预计算 |
| `endTime` | 仿真终点，默认参考关机时刻；短时检查可设30 |
| `referenceFile` | 成功PSO参考MAT文件 |

MPC结果和规划结果分别保存；一次控制仿真不会覆盖全局PSO参考。
