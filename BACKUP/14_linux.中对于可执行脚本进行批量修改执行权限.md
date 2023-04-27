# [linux 中对于可执行脚本进行批量修改执行权限](https://github.com/shu1ong/gitblog/issues/14)

善用find指令
```
chmod +x -Rv `find . -name "*.sh"`
```