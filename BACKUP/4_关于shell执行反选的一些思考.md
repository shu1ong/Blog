# [关于shell执行反选的一些思考](https://github.com/shu1ong/gitblog/issues/4)

## 背景
在使用openfoam的时候需要执行多次对于同一个案例，而需要保存每一次的结果。
目前的策略是对于不同的几何模型（网格）建立不同的工作文件夹。而多次运行算例的情况则使用shell对每次的结果进行归档。
剩下openfoam的初始运行环境为下一次的运行作准备。
典型的openfoam配置如下：
![image](https://user-images.githubusercontent.com/127008177/224473085-86aa667a-fb69-476f-9eda-8c952f81a0a0.png)
另外还要保留的是我们的脚本shell文件

网上一般反选mv的命令有两种
## 