# mgc3130 arduino lib
- 对DFRobot的arduino手势识别库做了修改

# 关于mgc3130
- datasheet一图解释
- 在片上flash集成了GestIC library Interface，里面已经有相关识别算法，用到了HMM，已经对数据做了处理和判断，用I2C读出来即可
- GestIC library Interface可以参看《MGC3030/3130 GestIC® Library Interface Description User’s Guide》


# 相关函数
- Wire.requestFrom(address,reclength)： I2C从address申请reclength个字节
