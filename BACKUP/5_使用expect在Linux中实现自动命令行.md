# [使用expect在Linux中实现自动命令行](https://github.com/shu1ong/gitblog/issues/5)

# 问题

在使用construct2d的时候，程序本身由Fortran编写需要改写的参数众多。需要一个键改写参数的脚本帮助我们快速实现几何文件的构建。

但construct2d是基于可执行文件的非shell菜单命令所以得想个办法实现参数的自动输入。

# 实现

### 方法1

echo大法 

```shell
echo -e "parameter\n" | ./shell.sh
```

这个办法需要shell脚本的嵌套，且只是字符串的简单叠加，必须一次装载。略显麻烦所以舍去。

### 方法2

expect

首先需要安装包

```shell
sudo apt install expect
```

 使用`spwan`启动程序

用expect去检测待输入的状态，参数可以写在文件的开头方便我们自行配置，如下：

expect的最后得有expect eof 停止等待的指令，否则无法根据字符进行检测

```shell
#!/usr/bin/expect
#sopt
set NSRF 300
set LESP 3.5E-03
set TESP 2.2E-04
set RADI 12
#vopt
set JMAX 110
set YPLS 0.95
set RECD 7.35E+05
set CFRC 1

spawn ./construct2d naca0012.dat

expect " Command >"
send "sopt\r"
#nsrf
expect " Input > "
send "NSRF\r"
expect " New value > "
send "$NSRF\r"
#LESP
expect " Input > "
send "LESP\r"
expect " New value > "
send "$LESP\r"
#TESP
expect " Input > "
send "TESP\r"
expect " New value > "
send "$TESP\r"
#RADI
expect " Input > "
send "RADI\r"
expect " New value > "
send "$RADI\r"

expect " Input > "
send "quit\r"
expect " Command >"
send "vopt\r"
#JMAX
expect " Input > "
send "JMAX\r"
expect " New value > "
send "$JMAX\r"
#YPLS
expect " Input > "
send "YPLS\r"
expect " New value > "
send "$YPLS\r"
#RECD
expect " Input > "
send "RECD\r"
expect " New value > "
send "$RECD\r"
#CFRC
expect " Input > "
send "CFRC\r"
expect " New value > "
send "$CFRC\r"

expect " Input > "
send "quit\r"
expect " Command >"
send "grid\r"
expect " Input > "
send "smth\r"
expect " Command >"
send "quit\r"
expect eof



```

