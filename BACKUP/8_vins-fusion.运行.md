# [vins-fusion 运行](https://github.com/shu1ong/gitblog/issues/8)

```shell
jsl@jsl-ThinkBook-14-G4-IAP:~/XTDrone/sensing/slam/vio$ python vins_transfer.py iris 0
Traceback (most recent call last):
  File "vins_transfer.py", line 6, in <module>
    import tf
  File "/opt/ros/noetic/lib/python3/dist-packages/tf/__init__.py", line 30, in <module>
    from tf2_ros import TransformException as Exception, ConnectivityException, LookupException, ExtrapolationException
  File "/opt/ros/noetic/lib/python3/dist-packages/tf2_ros/__init__.py", line 38, in <module>
    from tf2_py import *
  File "/opt/ros/noetic/lib/python3/dist-packages/tf2_py/__init__.py", line 38, in <module>
    from ._tf2 import *
ImportError: dynamic module does not define init function (init_tf2)
```

---

这个问题好像还是python混用的问题

---

https://blog.csdn.net/qq_39429669/article/details/121930799

> 问题原因

python版本错误

解决方法

打开文件夹/计算机/opt/ros/kinetic/lib/tf2_tools

在tf2_tools文件夹下打开终端，修改文件权限sudo  chmod 777 view_frames.py

打开文件view_frames.py，把第一行#!/usr/bin/env python修改为#!/usr/bin/env python2.7保存
————————————————
版权声明：本文为CSDN博主「qq_39429669」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_39429669/article/details/121930799