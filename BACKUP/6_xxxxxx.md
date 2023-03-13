# [xxxxxx](https://github.com/shu1ong/gitblog/issues/6)

CMake Error at /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
Could NOT find PythonInterp: Found unsuitable version "2.7.18", but   required is at least "3" 

这里是python的调用问题

因为ros1是支持python 2.0的 会有混用的情况的出现

目前的解决方案是在build目录下修改camkecache文件里
usr/bin/python
改为 
usr/bin/python3

---

/usr/bin/ld: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
make[2]: *** [CMakeFiles/MonoAR.dir/build.make:238：../MonoAR] 错误 1
make[1]: *** [CMakeFiles/Makefile2:487：CMakeFiles/MonoAR.dir/all] 错误 2
make: *** [Makefile:130：all] 错误 2


---

https://blog.csdn.net/lzRush/article/details/84579692

---

如果你是以CMakeList来管理项目，那么解决方法（以我的错误为例）如下：

1.打开CMakeList.txt

2.在CMakeList.txt中添加

//我缺少的是libopencv_imgproc.so.2.4这个链接库，我们只需要在CMakeList中定向链接这个库即可

set(LIB_OPENCV_IMGPROC_DIR /usr/local/lib)   //LIB_OPENCV_IMGPROC_DIR是变量名，可以随意写
add_library(libopencv_imgproc SHARED IMPORTED)
set_target_properties(libopencv_imgproc PROPERTIES IMPORTED_LOCATION ${LIB_OPENCV_IMGPROC_DIR}/libopencv_imgproc.so.2.4.9)

target_link_libraries(demo  libopencv_imgproc )//注意这条语句要放在add_executable的后面
————————————————
版权声明：本文为CSDN博主「御名方守矢-」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/lzRush/article/details/84579692

---

这个办法其实没有解决 最后解决的办法是修改了opencv的版本
参照
https://www.yuque.com/xtdrone/manual_cn/vslam
由于Noetic自带的OpenCV版本是4.2.0，在编译时需要先修改CMakeList.txt