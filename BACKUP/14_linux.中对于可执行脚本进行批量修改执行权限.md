# [linux 中对于可执行脚本进行批量修改执行权限](https://github.com/shu1ong/gitblog/issues/14)

善用find指令
```
chmod +x -Rv `find . -name "*.sh"`
```
提交超算指令
```
sbatch  sub.sh
```
查看作业队列
```
squeue
```
