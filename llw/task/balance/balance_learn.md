# 整体情况

| 功能             | 完成情况 | 说明 |
| ---------------- | -------- | ---- |
| 平衡小车URDF     |  已完成  | 在之前差速小车URDF基础上修改得来，调整了质量和对应的惯性矩阵 |
| 平衡小车关节驱动 |   已完成   | 通过给两边轮子加载力矩控制器实现 |
| 为平衡小车添加IMU  |  已完成  | 通过Gazebo传感器插件添加了 IMU |
| 搭建平衡小车控制器 | 已完成 | 实现了使用PID和LQR控制 |
| 添加云台 | 已完成 | 实现了跟随模式 |

## 功能细节

### 平衡小车URDF

![](../pic/balance_urdf_01.png)
如图所示，给平衡小车URDF设置了合适的惯性矩阵。

**遇到的问题** 由于当前车体质量分布情况，以及gazebo环境较理想，导致即使不加载控制器，小车也不会跌倒。

**我的解决方法** 通过加载时添加初始姿态参数，使小车一开始就处于倾斜状态。实践中，在让轮子转动后再停止，小车会进入倾倒状态。
```xml
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model balance_car -param robot_description -x 0 -y 0 -z 0.2 -P 0.1" />
```

> **注意！** 此处的小车urdf结构上存在问题无法控制平衡，具体来说**轮子的转轴不应与小车重心在同一水平线上**，而应比车体重心更低。后续会在控制器章节中进一步说明。


### 平衡小车关节驱动

![](../pic/balance_urdf_02.png)

这是一张运动中的截图。可从图中轮子坐标轴看出，轮子能够转动。

实现方式是通过给左右轮子分别加载力矩控制器，然后发布力矩指令即可。

**遇到的问题** 一开始发布力矩指令后，小车开始高速旋转，难以控制。

**我的解决方法** ：
- 首先认为是车身质量设置过低导致，调整质量并同步让ai生成惯性矩阵，问题仍存在。
- 后来发现是没有设置动力学参数，给轮子添加了适当的粘性阻尼和静摩擦力后，问题解决。
```xml
<dynamics damping="0.1" friction="0.1"/>
```

---

### 为平衡小车添加IMU

通过以下代码段添加了 imu 模块
```xml
    <gazebo reference="imu_link">
      <gravity>true</gravity>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <visualize>true</visualize>
        <imu/>
        <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
          <topicName>imu</topicName>
          <bodyName>imu_link</bodyName>
          <updateRateHZ>100.0</updateRateHZ>
          <gaussianNoise>0.0</gaussianNoise>
          <xyzOffset>0 0 0</xyzOffset>
          <rpyOffset>0 0 0</rpyOffset>
          <frameName>imu_link</frameName>
          <initialOrientationAsReference>false</initialOrientationAsReference>
        </plugin>
      </sensor>
    </gazebo>
```

### 搭建平衡小车控制器

#### 使用PID控制
**问题**：为了使小车实现平衡和其他的运动需求，我们需要控制哪些*状态*？

**我的理解**：需要控制小车 pitch 角，使其维持在一个目标值，即可实现平衡和其他运动需求。

参考文章：[平衡小车控制原理（受力分析及公式推导）](https://www.bilibili.com/read/cv7875274/?opus_fallback=1)

根据本文章我建立了对平衡小车控制的基本认知：**平衡就是控制 pitch 不变**
小车倾倒时 pitch 会增大，这是我们不希望看到的。只需要 pitch 固定在一个值，我们就可以认为小车是平衡的。

![](../pic/balance_car_01.png)
如上图所示，当平衡小车倾斜 $\theta$ 角时，重力产生偏离力矩 $M = L \cdot mg\sin\theta$。
为抵消此力矩，需控制轮子加速产生惯性力，形成反向平衡力矩 $M_1 = L_1 \cdot ma\cos\theta$。
由于 $L = L_1$，达到平衡的条件即为：$mg\sin\theta = ma\cos\theta$。

当 $\theta = 0$ 时，$M = 0$，处于静止或匀速运动的平衡态。

> 上面我们提到的小车urdf结构上存在问题: 若轮子转轴与重心重合，则 $L_1 = 0$ 导致 $M_1 \equiv 0$，轮子加速无法产生回复力矩。因此，**轮子转轴必须低于车体重心**。

现在回顾小车的运动需求：
- **小车能够自起**（指小车从倒在地上到平衡的状态）：将小车从倾倒状态（支持力矩平衡）变为由轮子加速度维持的动态平衡状态。
- **小车能够前后移动**：上面的推导中可以发现，若小车前倾，维持平衡需要轮子向前加速以产生反向力矩 $M_1$，这时小车整体会向前移动；反之亦然。所以我们可以通过主动打破 $\theta = 0$ 的平衡状态，使重心前偏移或后偏产生受力，进而**为维持小车 pitch 不变**，轮子就会加速驱动小车运行。

由此我们可以确立pid控制的思路：
1. 通过一个平衡环控制 $\theta$ 角，即 pitch 角，使其维持在目标 pitch 以确保平衡。
2. 通过一个速度环控制小车的目标 pitch，使小车能够前后移动。

以上两个环基本实现了前后移动和平衡、自起的运动目标，但对于小车能够在一定距离内稳定的要求，还需要增加一个位置环。综上可以得到以下的串级控制结构：

```mermaid
graph LR
    SetPos((设定位置)) --> PosLoop[位置环 PID]
    PosLoop -->|目标速度| VelLoop[速度环 PID]
    VelLoop -->|目标 Pitch| BalLoop[平衡环 PID]
    BalLoop -->|力矩指令| Motor[电机驱动]
    
    Motor --> Car{平衡小车}
    Car -.->|IMU 姿态反馈| BalLoop
    Car -.->|编码器速度反馈| VelLoop
    Car -.->|里程反馈| PosLoop
```

**遇到的问题**：即使添加了位置环，小车仍然无法稳定在一个位置，会缓慢来回移动。
**我的解决方法**：首先我尝试调节PID参数，尝试了将各 error 发布到话题，用 plotjuggler 看曲线的调试思路，但效果不明显。后面根据参考文章修改了控制公式，添加了*控制阻尼力*：
![](../pic/artical_01.png)

代码实现为：
```cpp
  //....
  current_pitch_ = pitch;
  current_pitch_dot_ = msg->angular_velocity.y; // 获取 IMU 提供的角速度作为微分项
  //....
  double pitch_error = current_pitch_ - target_pitch_;
  double base_effort = balance_pid_.computeCommand(pitch_error, current_pitch_dot_, period);
```

该方法让我产生如下思考：
- D 项是微分项，考虑物理量的关系，位置环的微分项是速度，速度环的微分项是加速度，平衡环的微分项是角速度。假设我在调一个控制角度的pid，那么相比起微分得到的d项，从**编码器**中读取角速度作为d项会更准确，避免微分产生的误差。
- I 项目是积分项，考虑物理量的关系，速度的积分是位置。假设我在调一个控制速度的pid，那么比起用 i 项来消除稳态误差，从车体别的接口中获取**位置相关信息**，外套一个位置环效果会更好（？）。

#### 使用LQR控制

##### 如何使用LQR

最直接地，我们需要建立小车的状态空间模型，包含一个状态矩阵 A 一个输入矩阵 B。接着通过选择合适的权重矩阵 Q 和 R，即可使用现成的轮子计算增益矩阵k
```python
control.lqr(A, B, Q, R)
```
增益矩阵k的四个元素分别对应模型中的四个状态变量。我们将之与对应的当前状态变量相乘并取负号，即可得到控制输入 u
```cpp
base_effort = -(k1_selfup * pos_error + k2_selfup * vel_error + k3_selfup * pitch_error + k4_selfup * omega_error);
```

##### 状态空间模型建立

![alt text](../pic/lqr_00.jpg)

**遇到的问题**：极性
**我的解决方法**：一定要注意状态变量的定义，尤其是角度的正负方向！而error的计算也要注意，lqr常用的是*当前状态减去目标状态*。

##### LQR 调参
LQR 调参的核心在于 Q 和 R 矩阵的选择。Q 矩阵中对角线元素越大，表示对应状态变量的重要性越高，系统会更倾向于减小该状态变量的误差。R 矩阵中对角线元素越大，表示对控制输入的惩罚越大，系统会更倾向于减少控制输入的使用。
实践上，当我们发现系统输出的力矩指令过大，超过了电机限制，可以考虑调小R矩阵的值。Q矩阵对角线元素的选取则依赖于具体的控制目标。

LQR的另一种调参思路是利用克雷森法则：[LQR控制器的参数整定技巧](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8ELQR%E6%8E%A7%E5%88%B6%E5%99%A8%E7%9A%84%E5%8F%82%E6%95%B0%E6%95%B4%E5%AE%9A%E6%8A%80%E5%B7%A7.md)
简单的讲，克雷森法则就是
- 将Q矩阵的各元素设定为该项可接受的最大误差的倒数平方。
- 将R矩阵的各元素设定为该项可接受的最大控制输入的倒数平方。

我在实践中运用此法则得到了不错的效果，使用的时候要注意各项目的单位都和推导过程中一样，即国际单位制。

**遇到的问题**：通过调整矩阵得到了不错的刹车效果，但无法倒地自起
**原因**：我认为是调大位置权重，导致系统过于关注位置误差，无法给予起身足够的位置误差。
**我的解决方法**：我开始尝试使用两套k，其中一套用于平衡和前后移动，另一套用于自起。通过构建一个倒地状态机来切换使用的k矩阵。
```cpp
  switch (current_state_) {
    case STATE_NORMAL:
        if (std::abs(pitch_error) > 0.835) {
            current_state_ = STATE_FALLEN;
            ROS_INFO("Switched to FALLEN state");
        }
        break;

    case STATE_FALLEN:
        if (std::abs(omega_error) < 0.2 && std::abs(vel_error) < 0.01) {
            current_state_ = STATE_SELF_UP;
            self_up_start_time_ = time;
            last_effort_ = 0.0;
            ROS_INFO("Switched to SELF_UP state");
        }
        break;

    case STATE_SELF_UP:
        if ((time - self_up_start_time_).toSec() > 6.0) {
            current_state_ = STATE_NORMAL;
            ROS_INFO("Switched to NORMAL state");
        }
        break;
  }
```

在给当前的小车加上一个pid控制yaw转向后，没有云台的平衡小车控制器部分就基本完成，控制效果良好，实现了所有运动需求。

### 添加云台

这次我添加了一个有yaw和pitch两个轴的云台
![alt text](../pic/balance_car_with_gimbal.png)

云台的yaw控制思路和先前差速小车的相同：云台的yaw轴pid跟随角速度指令，底盘的yaw轴pid跟随云台的yaw角度指令。

**遇到的问题**：添加云台后，小车的重心上移且会随云台移动而变化
**我的解决方法**：LQR算法具有一定的鲁棒性，通过调整Q和R矩阵的权重，能够适应云台带来的重心变化，保持小车的平衡控制效果。

**遇到的问题**：添加云台后，小车的平衡时pitch不为0而会有一个偏移
**我的解决方法**：通过计算这个偏移作为小车的目标pitch，从而补偿这个偏移，防止小车为控制pitch为0而飘车。