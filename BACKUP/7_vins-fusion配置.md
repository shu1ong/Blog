# [vins-fusion配置](https://github.com/shu1ong/gitblog/issues/7)

依赖
Ceres Solver
据说最好安装的版本是1.14.0
新的版本和eigen3.3.7好像不太匹配容易报错
安装指南 以及ceres的依赖
glog
https://zhuanlan.zhihu.com/p/459061947
ceres 1.14.0
https://zhuanlan.zhihu.com/p/460685629

---

靠 昨天装orb-slam2该了eigen3的版本 现在报错找不到包了

---

CMake Error at /usr/share/dart/cmake/DARTFindEigen3.cmake:12 (find_package):
  Could not find a package configuration file provided by "Eigen3" with any
  of the following names:

    Eigen3Config.cmake
    eigen3-config.cmake

---

![image](https://user-images.githubusercontent.com/127008177/224891038-e235f7e2-3a14-4f5a-84b7-c20bc74e42ff.png)
能够检验到包
考虑要不要改回去