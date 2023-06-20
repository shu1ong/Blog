# [关于pandas read_csv的一些小技巧](https://github.com/shu1ong/gitblog/issues/15)

1. 出现ParserError: Error tokenizing data的问题
一般是csv文件中，读取文件进行分割时出现问题
要么将文件转储为其他格式的文件，或者使用execel替换分隔符
2. path文件的路径下加r 可以识别路径中\ 和 /不匹配的问题