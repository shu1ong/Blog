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

---

![image](https://user-images.githubusercontent.com/127008177/224891603-83593070-25b0-47b3-850e-b4db10c111a1.png)
已经可以熟练的卸载重新安装了

---

![image](https://user-images.githubusercontent.com/127008177/224895429-9102bca5-d11e-41c0-a551-82585d1b9dab.png)
ceres又不行了，重新编译安装试试

---

成功解决以及新的问题：
![image](https://user-images.githubusercontent.com/127008177/224897678-a28b43e8-5fac-4ad4-ad4d-d87ad445f8e1.png)


---

本着缺什么补什么的精神
sudo apt-get install ros-noetic-cmake-modules

---

![image](https://user-images.githubusercontent.com/127008177/224903007-2ac48d3a-fa3e-44ec-8a61-76c5b5bed62e.png)
可以成功编译到84%

---

第一次报错的地方在这
![image](https://user-images.githubusercontent.com/127008177/224903863-b30f329e-3385-4e6c-8146-8453abd23325.png)
