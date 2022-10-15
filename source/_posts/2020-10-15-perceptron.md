---
title: "[机器学习] 感知机(Perceptron)&支持向量机(SVM)"
date: 2020-10-15 00:00:00
tags: 
    - 机器学习
categories:
    - 理论
katex: true
cover: false
---

# 感知机(Perceptron)

在特征空间中找到一个超平面w^Tx+b=0，使用该超平面将样本空间分割成两个部分，样本中属于这两个部分的点分别属于不同的正负两个类别。其中最为关键的部分应该为超平面的确定和优化。但是感知机对于线性不可分的样本并不能找到较好的决策面来分割样本，同时感知机所生成的超平面不一定是样本的最优分割。

决策面为：$Wx+b=0$

样本到决策面的距离：$r_i=\frac{W^Tx_i+b}{\left|\left|w\right|\right|}$

计算：略

# 支持向量机(SVM)

普通的SVM只对线性可分的数据集有比较好的结果，对于线性不可分的数据和普通的感知机一样也没有特别好的表现。同时经典的SVM只是一种二分类的方法，而实际环境中往往需要进行多分类，这时可以通过多个SVM组合来完成多分类，使用SVM决策树来解决问题。

SVM的original problem：
$\min\limits_{w,b}  \frac{1}{2}\|w\| \\ s.t.\ \ \ \ 1-y_i\left(wx_i\ +\ b\right)\ \le\ 0,\ i\ =\ 1,2...N$
这是一个QP问题，有维度d+1个变量参数和N个约束，可以使用一些求解QP问题的方法来求解。但是当引入核函数将特征空间映射到更高维的空间甚至是无限维的空间时这个问题可能无法求解。为此需要使用SVM问题的对偶形式，所以对于非线性SVM而言SVM的对偶问题很重要。

## 数学推导(不包括优化算法部分)

引入拉格朗日乘子：

$$\mathcal{L} \left(w,b,\alpha\right)=\frac{1}{2}\left|\left|w\right|\right|+\sum_{i=1}^{N}{\alpha_i\left(1-y_i\left(wx_i\ +\ b\right)\right)}$$

通过拉格朗日对偶性，将原始问题等价于一个极大极小问题：
$$\max\limits_{\alpha} \min\limits_{w,b}\mathcal{L} \left(w,b,\alpha\right)$$

再通过对w, b求梯度等于0，带入可得如下式子：
$$\min\limits_{w,b}\mathcal{L}\left(w,b,\alpha\right)=-\frac{1}{2}\sum_{i=1}^{N}\sum_{j=1}^{N}{\alpha_i\alpha_jy_iy_j\left(x_i\bullet x_j\right)}+\sum_{i=1}^{N}\alpha_i$$

再求得$\min\limits_{w,b}\mathcal{L}\left(w,b,\alpha\right)$对$\alpha$的极大即可：

$$\max\limits_\alpha\ \ -\frac{1}{2}\sum_{i=1}^{N}\sum_{j=1}^{N}{\alpha_i\alpha_jy_iy_j\left(x_i\bullet x_j\right)}+\sum_{i=1}^{N}\alpha_i$$
$$s.t.\sum_{i=1}^{N}{\alpha_iy_i}=0,\alpha_i\geq0,i=1,2,\cdots,N$$

这里的数据只出现了点乘，所以当引入核函数$\varphi\left(x\right)$时可以将其点乘$K\left(\cdot\right)$写成如下形式：

$$K\left(x_i,x_j\right)=\varphi\left(x_i\right)\varphi\left(x_j\right)$$

得SVM的对偶形式如下：

$$\max\limits_{\alpha} \ -\frac{1}{2}\sum_{i=1}^{N}\sum_{j=1}^{N}{\alpha_i\alpha_jy_iy_jK\left(x_i,x_j\right)}+\sum_{i=1}^{N}\alpha_i$$

接下来运用一些优化算法来计算$\alpha$即可，SVM常见的优化算法为SMO(序列最小优化)算法

### 优化算法部分参考

SMO算法参考文献：[http://cs229.stanford.edu/materials/smo.pdf](http://cs229.stanford.edu/materials/smo.pdf)