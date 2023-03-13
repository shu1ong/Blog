# [xxxxxx](https://github.com/shu1ong/gitblog/issues/6)

CMake Error at /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
Could NOT find PythonInterp: Found unsuitable version "2.7.18", but   required is at least "3" 

这里是python的调用问题

因为ros1是支持python 2.0的 会有混用的情况的出现

目前的解决方案是在build目录下修改camkecache文件里
usr/bin/python
改为 
usr/bin/python3