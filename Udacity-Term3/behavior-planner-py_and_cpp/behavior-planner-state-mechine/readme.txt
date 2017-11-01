状态机 states = ["KL", "LCL", "LCR", "PLCL", "PLCR"]

1. 删除边界位置的左转/右转状态
2.  每个state都模拟一遍，并计算cost
3. 采用cost最小的state作为下一个state

