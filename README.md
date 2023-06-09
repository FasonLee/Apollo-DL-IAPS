## 1.DL-IAPS & PJSO  
百度Apollo设计了一种横纵向解耦的OpenSpace规划算法（DL-IAPS & PJSO）  
横向（Dual-Loop Iterative Anchoring Path Smoothing）  
纵向（Piece-wise Jerk Speed Optimization）  
论文链接：https://arxiv.org/abs/2009.11135  

## 2.Apollo Discrete Points Smoother
百度Apollo参考线平滑算法，整体框架结构如下所示。  
![Image text](https://github.com/FasonLee/ApolloDiscretePointsSmoother/blob/master/pictures/ApolloReferenceLineSmooth.png)  


### *本项目是对上述（DL-IAPS & PJSO）算法及离散点平滑（Discrete Points Smoother）部分源码的移植。


## 3.三方库依赖
Ipopt  
Osqp-0.4.1  
Eigen3  
OsqpEigen  
Protobuf  
abseil-cpp  
Boost  
matplotlib-cpp  

## 4.参考链接
https://mp.weixin.qq.com/s/MwTVTHn5kK8c5a9PzyhD4Q  
https://zhuanlan.zhihu.com/p/342740447  
https://zhuanlan.zhihu.com/p/371585754  
https://zhuanlan.zhihu.com/p/325645742  
