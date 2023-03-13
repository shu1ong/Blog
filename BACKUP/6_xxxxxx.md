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