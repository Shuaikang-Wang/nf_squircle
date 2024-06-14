#!/bin/bash
# 定义一个变量，表示要执行的次数
n=10
# 定义一个变量，表示要执行的python文件的路径
file="/home/bnw/nf_lidar/nf-lidar/code/test_sensing.py"
# 使用for循环，从1到n，每次执行一次python文件
for i in $(seq 1 $n)
do
  echo "执行第$i次"
  python $file
done

