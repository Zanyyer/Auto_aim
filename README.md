# 从这开始

考核题目[click here](https://github.com/mechanical-ancestor/task7/tree/main)

### 自瞄项目目录结构

项目结构的搭建基本参考[同济开源](https://github.com/TongjiSuperPower/sp_vision_25)的方式

```bash
.
├── README.md    
│  
├── auto_aim/                   # 自瞄核心功能部分
│   ├── include
│   │   ├── detector.hpp
│   │   ├── solver.hpp
│   │   ├── tracker.hpp
│   │   └── type.hpp    
│   └── src
│       ├── detector.cpp
│       ├── solver.cpp
│       └── tracker.cpp 
│  
├── calibration/                # 标定
│   ├── camera_calib.cpp
│   └── capture.cpp
│            
├── configs/                   # 全部参数配置
│   └── ...
│
├── io/                         # 硬件层
│   ├── camera.hpp
│   ├── serials
│   │   ├── serial.cpp
│   │   └── serial.hpp
│   └── usbcamera
│       ├── usbcamera.cpp
│       └── usbcamera.hpp
│
├── tests/                     # 测试代码区               
│   └── ...
└── src/                       # 主程序源文件
    └── main.cpp              
```

### 项目简要介绍

- `io/`：目前仅实现了**USB相机**和**电脑间通讯**的功能，如果日后还有需求可以增加相应的内容
- `calibration/`：用于**相机标定**的独立功能区，想了解详情可以到文件夹里`README.md`查看
- `auto_aim/`：实现初步**自瞄**核心的功能区，下面将会简单介绍

#### 自瞄核心

##### 特征识别 *

参考[中南开源](https://github.com/CSU-FYT-Vision/FYT2024_vision)，将识别所需的装甲板和灯条属性整合到`type.hpp`文件，即将特征识别与解算甚至将来所需的其他功能共需的类型结合，在把各个功能单需的独立到各自的类，个人认为这样可以使得代码的灵活性与复用性提升。

其次仅简单完成了`detector.cpp`的一些功能，代码部分还有非常大的可优化空间，下面是一些可被优化的问题
1. 形态学操作的灵活性不足，即受环境和装甲板距离影响大
2. 匹配的灯条上下定点易飘，即容易抖动不稳定
  作者留言：这一点经过参考Chenjunnn的开源，得以解决（可重点查看`auto_aim/include/type.hpp`中Light的修改）如果想更进一步了解可以去看[Chenjunnn的开源](https://github.com/chenjunnn/rm_auto_aim?tab=readme-ov-file)。
3. 预测不够精准，结果绘制也不够明显
  修改了`solver.cpp`与`tracker.cpp`使得绘制结果明显,从只绘制中心点改为同时绘制**矩形+中心点**
4. 参数不精准
5. 算法逻辑可能不够严谨，目前仅是可以跑通

##### 位姿解算 *

此部分的**逻辑和架构**都是参考[同济开源](https://github.com/TongjiSuperPower/sp_vision_25)，**数据来源和方法**参考[Chenjunnn的开源](https://github.com/chenjunnn/rm_auto_aim?tab=readme-ov-file)（但此项目并不需要ROS，因此相机坐标的建立并不同）其他对应的知识可以看[OpenCV教程](https://docs.opencv.org/4.x/dc/d2c/tutorial_real_time_pose.html)

相机坐标轴：
- 原点：相机的光心（镜头中心）
- 坐标轴方向：
  - Z轴：沿光轴方向，指向前方（也就是物体在相机前方时，Z > 0）
  - X轴：水平向右
  - Y轴：竖直向下
装甲板坐标轴：
- 原点：装甲板中心
- 坐标轴方向：
  - Z轴：Down(竖直向下)
  - X轴：Forward(垂直与装甲板平面)
  - Y轴：Right(右)

##### 卡尔曼滤波 *

- 需要先学习或了解有关**线性代数**或**矩阵运算**的基本知识
- 了解卡尔曼滤波及其推导可以看B站博主[DR_CAN](https://www.bilibili.com/video/BV1yV411B7DM?spm_id_from=333.788.videopod.sections)，或者你有更好的建议可以issues

###### 重要内容

- 一些矩阵操作：
  - 转置：H.t() 
  - 求逆：S.inv()
  - 0矩阵： zeros()
  - 单位矩阵： eye()
  - 索引： at<T>(row, col)

- 参数

1. 状态矩阵 x_ (6*1)
2. 测量矩阵 z (3*1)
3. 状态转移矩阵 F(6*6)

```cpp
// CV_64F 64位浮点数

[ 0 ] /*x*/     [ 0 ]     [1  0  0  dt 0  0 ] // x方向：x[t+dt] = x[t] + v_x * dt  
[ 0 ] /*y*/     [ 0 ]     [0  1  0  0  dt 0 ]
[ 0 ] /*z*/     [ 0 ]     [0  0  1  0  0  dt]
[ 0 ] /*v_x*/             [0  0  0  1  0  0 ] 
[ 0 ] /*v_y*/             [0  0  0  0  1  0 ] 
[ 0 ] /*v_z*/             [0  0  0  0  0  1 ] 
```

4. 过程噪声协方差矩阵 Q_ (6*6)
5. 观测噪声协方差矩阵 R_ (3*3)
6. 转换矩阵 H (3*6)

用于将 6*1的状态矩阵 转换为 3*1的测量矩阵的形式

```cpp
 [1  0  0  0  0  0 ]     [1  0  0 ]     [1  0  0  0  0  0]
 [0  1  0  0  0  0 ]     [0  1  0 ]     [0  1  0  0  0  0]
 [0  0  1  0  0  0 ]     [0  0  1 ]     [0  0  1  0  0  0]
 [0  0  0  1  0  0 ] 
 [0  0  0  0  1  0 ] 
 [0  0  0  0  0  1 ] 
```

- 五个主要公式
  - 先验估计
    - x_ = F * x_  (6x1 = 6x6 * 6x1)
  - 先验误差协方差矩阵
    -  P_ = F * P_ * F.t() + Q_  (6x6 = 6x6 * 6x6 * 6x6 + 6x6)
  - 卡尔曼增益:
    - S = H * P_ * H.t() + R_   (3x3 = 3x6 * 6x6 * 6x3 + 3x3)
    - K = P_ * H.t() * S.inv()  (6x3 = 6x6 * 6x3 * 3x3) 
  - 后验估计: 
    - y = z - H * x_   (3x1 = 3x1 - 3x6 * 6x1)
    - x_ = x_ + K * y  (6x1 = 6x1 + 6x3 * 3x1)
  - 后验误差协方差矩阵
    - P_ = (I - K * H) * P_  (6x6 = (6x6 - 6x3 * 3x6) * 6x6)

### 感谢

#### GitHUb
- [Chenjunnn开源](https://github.com/chenjunnn/rm_auto_aim?tab=readme-ov-file)
- [同济开源](https://github.com/TongjiSuperPower/sp_vision_25)
- [中南开源](https://github.com/CSU-FYT-Vision/FYT2024_vision)

#### Bilibili
- [DR_CAN](https://www.bilibili.com/video/BV1yV411B7DM?spm_id_from=333.788.videopod.sections)
- [OpenCV实战](https://www.bilibili.com/video/BV1erWmzPE54/?spm_id_from=333.337.search-card.all.click)

#### OpenCV 
- [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html)
