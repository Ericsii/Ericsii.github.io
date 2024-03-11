---
title: >-
  论文阅读A Multi-State Constraint Kalman Filter for Vision-aided Inertial
  Navigation
comments: true
date: 2024-03-07 16:17:18
tags:
  - SLAM
  - 算法
categories:
  - 理论
description:
cover: false
katex: true
---

# [MSCKF] A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation

这篇06年的文章提出了一种新的几何约束，将3D feature的位置信息从卡尔曼滤波中的观测模型中去除，从而减少计算复杂度。

## Related works

> the main limitation of SLAM is its high computational complexity; properly treating these correlations is computationally costly, and thus performing vision-based SLAM in environments with thousands of features remains a challenging problem.

该文章也采用了部分使用图像之间对极约束的方法^[Diel, DavidD. Stochastic Constraints for Vision-Aided Inertial Navigation. Jan. 2005.]^[Bayard, D. S., and P. B. Brugarolas. “An Estimation Algorithm for Vision-Based Exploration of Small Bodies in Space.” Proceedings of the 2005, American Control Conference, 2005., 2005, https://doi.org/10.1109/acc.2005.1470719.]，不同的是该文章的 **MSCKF** 可以在多个相机 pose 之间计算约束。

> one fundamental difference is that our algorithm can express constraints between multiple camera poses, and can thus attain higher estimation accuracy, in cases where the same feature is visible in more than two images.

同样的多个相机 pose 同时优化的方法^[Eustice, Ryan, et al. “Visually Navigating the RMS Titanic with SLAM Information Filters.” Robotics: Science and Systems I, 2016, https://doi.org/10.15607/rss.2005.i.008.]，一个很大的缺点是当单个 feature 被多个图像观测时，其中多 pose 之间的约束被忽略了导致信息的缺失。

> Moreover, in contrast to SLAM-type approaches, it does not require the inclusion of the 3D feature positions in the filter state vector, but still attains optimal pose estimation. As a result of these properties, the described algorithm is very efficient.

## EKF 设计

### 状态空间

$$\mathbf{X}_\mathrm{IMU}=\begin{bmatrix}^I_G\bar{q}^T&\mathbf{b}_g^T&^G\mathbf{v}_I^T&\mathbf{b}_a^T&^G\mathbf{p}_I^T\end{bmatrix}^T$$

该文章使用$^I_G\bar{q}$这个四元数来表示用IMU系到G系(Global系)， $\mathbf{p}_I$ 和 $^G\mathbf{v}_I$ 表示 IMU 的在G系的位置和速度，$\mathbf{b}_g^T$ 和 $\mathbf{b}_a$ 是陀螺仪和加速度计的偏移。

根据上式写出IMU误差状态：

$$\widetilde{\mathbf{X}}_{\mathrm{IMU}}=\begin{bmatrix}\delta\boldsymbol{\theta}_{I}^T&\widetilde{\mathbf{b}}_{g}^T&^G\widetilde{\mathbf{v}}_{I}^T&\widetilde{\mathbf{b}}_{a}^T&^G\widetilde{\mathbf{p}}_{I}^T\end{bmatrix}^T$$

其中
$$\delta\bar{q}~~\simeq~~\left[\frac{1}{2}\delta\theta^{T}~~~1\right]^{T}$$

根据以上可以写出EKF的状态向量，将有$N$个相机pose状态和IMU状态的合并起来：

$$\hat{\mathbf{X}}_k=\begin{bmatrix}\hat{\mathbf{X}}_{\mathrm{IMU}_k}^T&^{C_1}_G\hat{\hat{q}}^T&^G\hat{\mathbf{p}}_{C_1}^T&\cdots&^{C_N}_G\hat{\mathbf{q}}^T&^G\hat{\mathbf{p}}_{C_N}^T\end{bmatrix}^T$$

$C_i$ 代表第 $i$ 个相机的相机系。

### 连续时间建模

IMU的系统可以用以下的微分方程描述：

$$_G^I\dot{\bar{q}}(t)=\frac{1}{2}\boldsymbol{\Omega}(\boldsymbol{\omega}(t))_G^I\bar{q}(t)$$

$$\dot{\mathbf{b}}_g(t)=\mathbf{n}_{wg}(t)$$

$$^G\dot{\mathbf{v}}_I(t)=^G\mathbf{a}(t),\dot{\mathbf{b}}_a(t)=\mathbf{n}_{wa}(t),^G\dot{\mathbf{p}}_I(t)=^G\mathbf{v}_I(t)$$

其中$\Omega(\omega)$为四元数增量：

$$\Omega(\omega)=\begin{bmatrix}-\lfloor\omega\times\rfloor&\omega\\-\omega^T&0\end{bmatrix},\quad\lfloor\omega\times\rfloor=\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix}$$

IMU的测量和状态的关系如下：

$$\begin{aligned}&\mathbf{\omega}_m=\mathbf{\omega}+\mathbf{C}(_G^I\bar{q})\mathbf{\omega}_G+\mathbf{b}_g+\mathbf{n}_g\\&\mathbf{a}_m=\mathbf{C}(_G^I\bar{q})(^G\mathbf{a}-{}^G\mathbf{g}+2\lfloor\mathbf{\omega}_G\times\rfloor^G\mathbf{v}_I+\lfloor\mathbf{\omega}_G\times\rfloor^2{}^G\mathbf{p}_I)+\mathbf{b}_a+\mathbf{n}_a\end{aligned}$$

$C(\cdot)$是旋转矩阵。

将代入每个状态的均值代入系统方程：

$$^I_G\dot{\hat{\bar{q}}}=\frac{1}{2}\boldsymbol{\Omega}(\hat{\boldsymbol{\omega}})_G^I\hat{\hat{q}},\quad\dot{\hat{\boldsymbol{b}}}_g=\boldsymbol{0}_{3\times1}$$

$$^G\dot{\hat{\mathbf{v}}}_I=\mathbf{C}_{\hat{\boldsymbol{q}}}^T\hat{\mathbf{a}}-2\lfloor\omega_G\times\rfloor^G\hat{\mathbf{v}}_I-\lfloor\omega_G\times\rfloor^2{}^G\hat{\mathbf{p}}_I+{}^G\mathbf{g}$$

$$\dot{\hat{\mathbf{b}}}_a=\mathbf{0}_{3\times1},^G\dot{\hat{\mathbf{p}}}_I=^G\hat{\mathbf{v}}_I$$

其中 $\mathbf{C}_{\hat{q}}=\mathbf{C}({}_{G}^{I}\hat{\bar{q}}),\hat{\mathbf{a}}=\mathbf{a}_{m}-\hat{\mathbf{b}}_{a},\hat{\boldsymbol{\omega}}=\omega_{m}-\hat{\mathbf{b}}_{g}-\mathbf{C}_{\hat{q}}\omega_{G}$。
初看 $^G\dot{\hat{\mathbf{v}}}_I$ 这一行的时候会很疑惑，后面得知中间的两项分别是，柯氏力加速度：$-2\lfloor\omega_G\times\rfloor^G\hat{\mathbf{v}}_I$，向心加速度：$-\lfloor\omega_G\times\rfloor^2{}^G\hat{\mathbf{p}}_I$。

之后我们可以得到IMU的误差状态方程：

$$\begin{array}{rcl}\dot{\widetilde{\mathbf{x}}}_\mathrm{IMU}&=&\mathbf{F}\widetilde{\mathbf{X}}_\mathrm{IMU}+\mathbf{G}\mathbf{n}_\mathrm{IMU}\end{array}$$

$$\mathbf{n}_\mathrm{IMU}=\begin{bmatrix}\mathbf{n}_g^T&\mathbf{n}_{wg}^T&\mathbf{n}_a^T&\mathbf{n}_{wa}^T\end{bmatrix}^T$$

$$\mathbf{F}=\begin{bmatrix}-\lfloor\hat{\boldsymbol{\omega}}\times\rfloor&-\mathbf{I}_3&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\\-\mathbf{C}_{\hat{\boldsymbol{q}}}^T\lfloor\hat{\boldsymbol{a}}\times\rfloor&\mathbf{0}_{3\times3}&-2\lfloor\omega_G\times\rfloor&-\mathbf{C}_{\hat{\boldsymbol{q}}}^T&-\lfloor\omega_G\times\rfloor^2\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{I}_3&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\end{bmatrix}$$

$$\mathbf{G}=\begin{bmatrix}-\mathbf{I}_3&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\\\mathbf{0}_{3\times3}&\mathbf{I}_3&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&-\mathbf{C}_{\hat{q}}^T&\mathbf{0}_{3\times3}\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{I}_3\\\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times3}\end{bmatrix}$$

### 离散时间状态

令状态的协方差矩阵为：

$$\mathbf{P}_{k|k}=\begin{bmatrix}
  \mathbf{P}_{II_{k|k}}   & \mathbf{P}_{IC_{k|k}} \\
  \mathbf{P}_{IC_{k|k}}^T & \mathbf{P}_{CC_{k|k}}
\end{bmatrix}$$

$\mathbf{P}_{II_{k|k}}$ 为一个 $15\times 15$ 的矩阵代表 IMU 状态的协方差，$\mathbf{P}_{CC_{k|k}}$ 是 $6N\times 6N$ 的相机pose状态协方差矩阵，$\mathbf{P}_{IC_{k|k}}$ 是 IMU 与相机 pose 之间的协方差矩阵。这样状态传播的协方差矩阵就如下式所示：

$$\mathbf{P}_{k+1|k}=\begin{bmatrix}\mathbf{P}_{II_{k+1|k}}&\mathbf{\Phi}(t_k+T,t_k)\mathbf{P}_{IC_{k|k}}\\\mathbf{P}_{IC_{k|k}}^T\mathbf{\Phi}(\iota_k+T,\iota_k)^T&\mathbf{P}_{CC_{k|k}}\end{bmatrix}$$

论文中描述到$\mathbf{P}_{II_{k+1|k}}$是通过Lyapunov equation在 $(t_k, t_k + T)$ 时段中数值积分来计算到的。Lyapunov equation: $\dot{\mathbf{P}}_{II}=\mathbf{F}\mathbf{P}_{II}+\mathbf{P}_{II}\mathbf{F}^T+\mathbf{G}\mathbf{Q}_{\mathrm{IMU}}\mathbf{G}^T$。状态转移矩阵 $\Phi(t_k + T, t_k)$ 同样是由数值积分得出的：

$$\dot{\Phi}(t_k+\tau,t_k)=\mathbf{F}\mathbf{\Phi}(t_k+\tau,t_k),\quad\tau\in[0,T]$$

### 状态增广

$$
_G\hat{\bar{q}}=_I^C\bar{q}\otimes_G^I\hat{\bar{q}}
$$
$$^G\hat{\mathbf{p}}_C=^G\hat{\mathbf{p}}_I+\mathbf{C}_{\hat{q}}^{TI}\mathbf{p}_C
$$

这是将相机坐标系中的点转换到Global系中，按照以上的方程EKF中协方差矩阵的转移如下：

$$\mathbf{P}_{k|k}\leftarrow\begin{bmatrix}\mathbf{I}_{6N+15}\\\mathbf{J}\end{bmatrix}\mathbf{P}_{k|k}\begin{bmatrix}\mathbf{I}_{6N+15}\\\mathbf{J}\end{bmatrix}^T$$

$$
\mathbf{J}=\begin{bmatrix}
\mathbf{C}\left(^C_I\bar{q}\right)&\mathbf{0}_{3\times9}&\mathbf{0}_{3\times3}&\mathbf{0}_{3\times6N}\\\lfloor\mathbf{C}_{\hat{q}}^{TI}\mathbf{p}_C\times\rfloor&\mathbf{0}_{3\times9}&\mathbf{I}_3&\mathbf{0}_{3\times6N}
\end{bmatrix}
$$

### 观测模型

对于误差状态 $\tilde{\mathbf{X}}$ 的观测残差为：

$$\mathbf{r=H\widetilde{X}+noise}$$

$H$ 为观测方程的 Jacobian 矩阵，噪声项 noise 必须为均值为0且与状态不相关的高斯分布。

整个观测模型的建模方式在目前来看比较直观，按照每个 tracked feature 对 camera pose 进行分组约束以此来使用单个 feature 同时约束多个相机 pose 的相对位姿，同时后续的推导中可以使约束方程中不含 feature 的 3D position 信息。

观测的大致方式是将 feature 的 3D position 投影到相机成像平面坐标并与图像中跟踪的 feature 图像坐标计算残差。

$$\mathbf{z}_i^{(j)}=\dfrac{1}{^{C_i}Z_j}\begin{bmatrix}^{C_i}X_j\\^{C_i}Y_j\end{bmatrix}+\mathbf{n}_i^{(j)},\quad i\in\mathcal{S}_j$$

$$^{C_i}\mathbf{p}_{f_j}=\begin{bmatrix}^{C_i}X_j\\^{C_i}Y_j\\^{C_i}Z_j\end{bmatrix}=\mathbf{C}(_G^{C_i}\bar{q})(^G\mathbf{p}_{f_j}-^G\mathbf{p}_{C_i})$$

上式代表将在全局坐标系中的 3D feature 位置 $^G\mathbf{p}_{f_j}$ 转到相机系。因为初始的情况下 3D feature 未知，该文章使用了最小二乘的方式先估计$f_j$的位置$^G\hat{\mathbf{p}}_{f_j}$(**该方法在论文的附录中**)。之后可以构建出观测的残差：

$$\mathrm{r}_i^{(j)}=\mathrm{z}_i^{(j)}-\hat{\mathrm{z}}_i^{(j)}$$

$$\hat{\mathbf{z}}_i^{(j)}=\dfrac{1}{C_i\hat{Z}_j}\begin{bmatrix}C_i\hat{X}_j\\C_i\hat{Y}_j\end{bmatrix},\begin{bmatrix}C_i\hat{X}_j\\C_i\hat{Y}_j\\C_i\hat{Z}_j\end{bmatrix}=\mathbf{C}(\begin{matrix}C_i\hat{\vec{q}}\\q\end{matrix})(^G\mathbf{\hat{p}}_{f_j}-{}^G\mathbf{\hat{p}}_{C_i})$$

上式通过泰勒展开之后的一阶近似可以写成：

$$\mathrm{r}_i^{(j)}\simeq\mathrm{H}_{\mathbf{X}_i}^{(j)}\widetilde{\mathbf{X}}+\mathrm{H}_{f_i}^{(j)G}\widetilde{\mathbf{p}}_{f_j}+\mathbf{n}_i^{(j)}$$

$\widetilde{\mathbf{p}}_{f_j}$ 是 $f_j$ 的位置误差。

$$\mathbf{H}_{\mathbf{X}_i}^{(j)}=\begin{bmatrix}\mathbf{0}_{2\times15}&\mathbf{0}_{2\times6}&\ldots&\underbrace{\mathbf{J}_i^{(j)}\lfloor^{C_i}\hat{\mathbf{X}}_{f_j}\times\rfloor -\mathbf{J}_i^{(j)}\mathbf{C}\binom{C_i}G}_{\text{Jacobian wrt pose i}}&\ldots\end{bmatrix}$$

$$\mathbf{H}_{f_i}^{(j)}=\mathbf{J}_i^{(j)}\mathbf{C}(_G^{C_i}\hat{\bar{q}})$$

$$\mathbf{J}_i^{(j)}=\nabla_{c_{i\hat{\mathbf{p}}f_j}}\mathbf{z}_i^{(j)}=\frac{1}{C_i\hat{Z}_j}\begin{bmatrix}1&0&-\frac{C_i\hat{X}_j}{C_i\hat{Z}_j}\\0&1&-\frac{C_i\hat{Y}_j}{C_i\hat{Z}_j}\end{bmatrix}$$

将所有观测 $M_j$ 全都整合成矩阵形式：

$$\mathrm{r}^{(j)}\simeq\mathrm{H}_{\mathbf{X}}^{(j)}\widetilde{\mathbf{X}}+\mathrm{H}_{f}^{(j)G}\widetilde{\mathbf{p}}_{f_{j}}+\mathbf{n}^{(j)}$$

其中 $\mathbf{r}^{(j)}, \mathbf{H}_{\mathbf{X}}^{(j)}, \mathbf{H}_{f}^{(j)G}, \mathbf{n}^{(j)}$ 是将 $\mathbf{r}_i^{(j)}, \mathbf{H}_{\mathbf{X}_i}^{(j)}, \mathbf{H}_{f_i}^{(j)}$ 堆叠形成的分块向量和矩阵。需要注意的是：**不同图像中观测到的feature是独立的**，因此噪声 $\mathbf{n}^{(j)}$ 的协方差为 $\mathbf{R}^{(j)}=\sigma^2 \mathbf{I}_{2M_j}$。

一个很大的问题是计算误差 $\widetilde{\mathbf{p}}_{f_j}$ 的过程中需要用到状态 $\mathbf{X}$ ，因此 $\widetilde{\mathbf{p}}_{f_j}$ 与 $\tilde{\mathbf{X}}$ 是**相关的**。所以该式实际上与$\mathbf{r=H\widetilde{X}+noise}$的形式不符。

为了解决这个问题 MSCKF 中将误差 $\mathbf{r^{(j)}}$ 投影到 $\mathbf{H}_f^{(j)}$ 的左零空间中，左零空间的定义是 $\mathcal{N}_{left}(\mathbf{H}_f^{(j)}) = \left\{ x \in \mathbb{R}^n | x^T \mathbf{H}_f^{(j)} = \mathbf{0} \right\}$ 其中 $n$ 为矩阵的行数。令矩阵 $\mathbf{A}$ 的列空间为张成 (Span) 空间 $\mathcal{N}_{left}(\mathbf{H}_f^{(j)})$ 的一个酉矩阵，这样误差方程可以写成：

$$\begin{aligned}
\mathbf{r}_{o}^{(j)}& =\mathbf{A}^{T}(\mathbf{z}^{(j)}-\hat{\mathbf{z}}^{(j)})\simeq\mathbf{A}^{T}\mathbf{H}_{\mathbf{X}}^{(j)}\widetilde{\mathbf{X}} + \underbrace{\mathbf{A}^T\mathbf{H}_f^{(j)G}\widetilde{\mathbf{p}}_{f_{j}}}_{\mathbf{0}} +\mathbf{A}^{T}\mathbf{n}^{(j)}  \\
&=\mathbf{H}_o^{(j)}\widetilde{\mathbf{X}}^{(j)}+\mathbf{n}_o^{(j)}
\end{aligned}$$

文章中分析到：$\mathbf{H}_f^{(j)}$(维度$2M_j\times3$)为列满秩，秩为3，因此其左零空间的维度为 $2M_j - 3$ ，因此 $\mathbf{r}_{o}^{(j)}$ 的维度为 $2M_j - 3$。可以看到，$\mathbf{r}_{o}^{(j)}$ 的式子中并不包含 feature 的 3D position 信息，仅仅是经过线性化的 pose 约束，可以直接用于 EKF 的 update。

根据^[Wilkinson, J. H., and CleveB. Moler. “Matrix Computations.” Encyclopedia of Computer Science,Encyclopedia of Computer Science, Jan. 2003.]，矩阵$\mathbf{A}$ 并不需要显式计算，使用吉文斯旋转(Givens rotations)方法求解，时间复杂度为 $O(M_j^2)$ 。

同时，因为 $\mathbf{A}$ 为酉矩阵，$\mathbf{n}_o^{(j)}$ 的协方差矩阵为：

$$E\{\mathbf{n}_o^{(j)}\mathbf{n}_o^{(j)T}\}=\sigma^2_{im}\mathbf{A}^T\mathbf{A} = \sigma^2_{im} \mathbf{I}_{2M_j-3}$$

文章中也提到，这并不唯一的一种残差表示形式。也可以使用对极约束来建模几何约束，实验表明使用对极约束会导致计算复杂度更高但是并没有得到更好的结果。

### EKF 状态更新

文章首先说明EKF的update会在以下几种情况下被触发：

- 当在多幅图像中被跟踪的特征不再被跟踪到时，该特征的所有测量值都使用上文中提出的方法进行处理。
- 当达到最大允许相机 pose $N_{max}$，至少有一个 pose 需要从状态中移除。MSCKF 中的策略是从第二老的 pose 开始均匀选择 $N_{max}/3$ 个 pose 保留并计算 update。**注意**: 总是保留最久的 pose 来保证几何约束中的基线足够长。

接下来详细看看 update 的数学形式，先将之前的残差项整合成矩阵形式：

$$\mathrm{r}_o=\mathrm{H}_\mathbf{X}\widetilde{\mathrm{X}}+\mathrm{n}_o$$

$\mathrm{r}_o, \mathrm{n}_o$ 是 $L$ 个 feature 组成， $\mathrm{r}_o^{(j)}, \mathrm{n}_o^{(j)}, j=1\ldots L$，同理 $\mathrm{H}_\mathbf{X}$ 为 $\mathbf{H}_o^{(j)}, j=1\ldots L$ 组成的分块矩阵。因为各个观测是线性无关的，所以 $\mathbf{n}_o$ 的协方差也可以写成单位矩阵的形式 $\mathbf{R}_{o}=\sigma_{\mathrm{im}}^{2}\mathbf{I}_{d}, d=\sum_{j=1}^{L}(2M_j - 3)$。

下一步对 $\mathrm{H}_\mathbf{X}$ 进行QR分解：

$$\mathbf{H_X}=\begin{bmatrix}\mathbf{Q}_1&\mathbf{Q}_2\end{bmatrix}\begin{bmatrix}\mathbf{T}_H\\0\end{bmatrix}$$

根据[QR分解](https://en.wikipedia.org/wiki/QR_decomposition#Rectangular_matrix)的说明，$\mathbf{Q}_1, \mathbf{Q}_2$ 为两个酉矩阵其列空间分别是 $\mathbf{H_X}$ 的像空间和零空间，$\mathbf{T}_H$ 为上三角矩阵。左乘 $[\mathbf{Q}_1, \mathbf{Q}_2]^T$ 我们可以将残差转化成紧QR分解(Thin QR factorization)形式：

$$\begin{aligned}
\mathbf{r}_{o}& =\begin{bmatrix}\mathbf{Q}_1&\mathbf{Q}_2\end{bmatrix}\begin{bmatrix}\mathbf{T}_H\\\mathbf{0}\end{bmatrix}\widetilde{\mathbf{X}}+\mathbf{n}_o\Rightarrow   \\
\begin{bmatrix}\mathbf{Q}_1^T\mathbf{r}_o\\\mathbf{Q}_2^T\mathbf{r}_o\end{bmatrix}& =\begin{bmatrix}\mathbf{T}_H\\0\end{bmatrix}\widetilde{\mathbf{X}}+\begin{bmatrix}\mathbf{Q}_1^T\mathbf{n}_o\\\mathbf{Q}_2^T\mathbf{n}_o\end{bmatrix} 
\end{aligned}$$

其中的 $\mathbf{Q}_2^T\mathbf{r}_o$ 为纯噪声项可以忽略，所以取上式中非0的部分：

$$\mathrm{r}_n=\mathbf{Q}_1^T\mathbf{r}_o=\mathrm{T}_H\widetilde{\mathbf{X}}+\mathrm{n}_n$$

噪声项的协方差为：$\mathbf{R}_n=\mathbf{Q}_1^T\mathbf{R}_o\mathbf{Q}_1=\sigma^2_{im}\mathbf{I}_r$， $r$ 为 $\mathbf{Q}_1$ 的列数。以上卡尔曼滤波的增益可以写出：

$$\mathbf{K}=\mathbf{P}\mathbf{T}_H^T\left(\mathbf{T}_H\mathbf{P}\mathbf{T}_H^T+\mathbf{R}_n\right)^{-1}$$

状态的修正增量为：

$$
\Delta \mathbf{X} = \mathbf{K} \mathbf{r}_n
$$

$$
\check{\mathbf{X}} = \tilde{\mathbf{X}} + \Delta \mathbf{X}
$$

协方差的传递写成：

$$\mathbf{P}_{k+1|k+1}=\left(\mathbf{I}_\xi-\mathbf{K}\mathbf{T}_H\right)\mathbf{P}_{k+1|k}\left(\mathbf{I}_\xi-\mathbf{K}\mathbf{T}_H\right)^T+\mathbf{K}\mathbf{R}_n\mathbf{K}^T$$

$\xi = 6N + 15$，QR分解可以使用吉文斯旋转复杂度为 $O(r^2d)$， EKF更新的复杂度为 $max\{O(r^2d), O(\xi^3)\}$。

## 总结

可以看到，整个MSCKF中最为核心的部分就是将残差方程中使用投影到零空间的方式消除其中各项的线性相关性，来满足EKF使用的条件，同时这种投影方式还直接简化了滤波中的状态空间，在当时看来状态空间中不包含feature的3D位置信息能极大的简化计算复杂度。
