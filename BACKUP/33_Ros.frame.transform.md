# [Ros frame transform](https://github.com/shu1ong/gitblog/issues/33)

- 坐标之间的转换通过TF工具实现
- TF的实现一般分为两种
    - 一种是写在launch文件中的,通过package进行实现
    - 另一种是写在程序里面,通过调用函数`sendtransform`实现