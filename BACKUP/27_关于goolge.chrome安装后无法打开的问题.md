# [关于goolge chrome安装后无法打开的问题](https://github.com/shu1ong/gitblog/issues/27)

主要是反映在root权限的问题
##解决办法
在路径`usr/bin`下找到你的google文件，使用sudo权限进行修改
在最后加上
```bash
--user-data-dir --no-sandbox
```