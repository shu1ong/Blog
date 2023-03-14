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