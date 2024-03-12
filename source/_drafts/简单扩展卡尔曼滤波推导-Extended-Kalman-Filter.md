---
title: 简单扩展卡尔曼滤波推导 Extended Kalman Filter
comments: true
tags:
  - 算法
  - SLAM
categories:
  - 理论
cover: false
katex: true
---

# 简单扩展卡尔曼滤波推导 Extended Kalman Filter

最近看了很多KF的推导，发现不同数学推导的形式差别很大，例如《概率机器人》中从贝叶斯滤波的形式推导，《视觉SLAM14讲》中用跳过了贝叶斯滤波的部分直接从高斯分布开始推导。优点是数学推导严谨，但是对于一些初学者而言理解起来较为费力。我本人本科并非控制，所以花了很长时间来理解卡尔曼滤波。

这里我想根据之前使用过的Python滤波器库 [FilterPy](https://github.com/rlabbe/filterpy) 的作者所著的这本 [Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) 中所描述的一种新的方式来理解卡尔曼滤波算法。

## 准备

首先需要理解的是什么叫滤波器