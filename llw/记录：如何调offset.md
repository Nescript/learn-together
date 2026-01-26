---
tags:
  - type/project-note
  - topic/robomaster
  - "#status/wip"
date: 2026-01-10
---
> 本笔记包含：调整 offset 的实际流程与连车相关操作
> 和 offset 有关的校准控制器的相关代码解读

调拨盘 offset 的方法 ：
首先为什么要调 offset？
我们希望拨盘能在校准后去到理想的发弹位置，但所采用的增量式编码器电机每次上电时的位置会成为新的零点，所以我们会通过校准控制器将其转到机械限位处，这是一个确定的位置。但注意！机械限位并不是理想的发弹位置，我们测量理想发弹位置和机械限位的距离差，将之设为 offset。

这样在每次校准后，拨盘顶到机械限位，随后控制器将当前位置加上 offset 作为电机 0 点，此时的 0 点就是理想的发弹位置，弹链不会太紧也不会太松，可以避免双发等问题。




调整的思路：一颗小弹丸送上去是 0.7 rad，这就相当于一个周期了。
load_controller load 所有的控制器，下一步就是通过 rqt 来 start 控制器
关自启的指令 sudo systemctl stop startmaster.service

要调节的位置：rm_description/urdf/(要调节的兵种)/shooter.transmission.urdf.xacro
下图所示的 offset

```xml
<transmission name="trans_trigger_joint">
    <type>transmission_interface/SimpleTransmission</type>
    <actuator name="trigger_joint_motor">
        <mechanicalReduction>-27.5</mechanicalReduction>
    </actuator>
    <joint name="trigger_joint">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <offset>0.355</offset>
    </joint>
</transmission>
```


```cpp
void MechanicalCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
    case MOVING_POSITIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_ && !actuator_.getHalted())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        if (!is_center_)
        {
          actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
          actuator_.setCalibrated(true);
          ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
          state_ = CALIBRATED;
          if (is_return_)
          {
            position_ctrl_.setCommand(target_position_);
          }
          else
          {
            velocity_ctrl_.joint_.setCommand(0.);
            calibration_success_ = true;
          }
        }
        else
        {
          positive_position_ = actuator_.getPosition();
          countdown_ = 100;
          velocity_ctrl_.setCommand(-velocity_search_);
          state_ = MOVING_NEGATIVE;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case MOVING_NEGATIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_)
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        negative_position_ = actuator_.getPosition();
        actuator_.setOffset(-(positive_position_ + negative_position_) / 2 + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
        if (is_return_)
          position_ctrl_.setCommand(target_position_);
        else
        {
          velocity_ctrl_.joint_.setCommand(0.);
          calibration_success_ = true;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
    case CALIBRATED:
    {
      if (is_return_)
      {
        if ((std::abs(position_ctrl_.joint_.getPosition() - target_position_)) < position_threshold_)
          calibration_success_ = true;
        position_ctrl_.update(time, period);
      }
      else
        velocity_ctrl_.update(time, period);
      break;
    }
  }
}
```
这是机械限位校准控制器的实现，以下给出解读
可以发现控制器采用了简单的 state_ 状态机，刚开始运行时
```cpp
case INITIALIZED:
    {
      velocity_ctrl_.setCommand(velocity_search_);
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
```
将电机速度设为搜索速度（在参数中定义），设置了 100 个周期的倒计时，并将状态切换为 `MOVING_POSITIVE` 即正向移动。

```cpp
case MOVING_POSITIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_ && !actuator_.getHalted())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)
      {
        velocity_ctrl_.setCommand(0);
        if (!is_center_)
        {
          actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
          actuator_.setCalibrated(true);
          ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
          state_ = CALIBRATED;
          if (is_return_)
          {
            position_ctrl_.setCommand(target_position_);
          }
          else
          {
            velocity_ctrl_.joint_.setCommand(0.);
            calibration_success_ = true;
          }
        }
        else
        {
          positive_position_ = actuator_.getPosition();
          countdown_ = 100;
          velocity_ctrl_.setCommand(-velocity_search_);
          state_ = MOVING_NEGATIVE;
        }
      }
      velocity_ctrl_.update(time, period);
      break;
    }
```
正向移动时，如果电机移动速度低于阈值，且控制器仍在运行，则认为目前电机发生**堵转**，开始倒计时。当堵转持续 100 个周期，则停止控制器并开始进一步处理
- 如果当前校准的控制器是要找中点，那么**还需要往反方向走**来找到另一个机械限位点，然后计算得出中点
- 如果当前校准的控制器不用找中点，那么就将电机的 offset 设为**当前位置取反加上 urdf 中设置的 offset** 即可
为什么是取反呢？
因为当电机去到堵转位置，理论上这就是零点。offset 设置的目的是使得电机 getPosition 后加上 offset 得到 0,所以 offset 应该设置为当前**负的**堵转位置的 getPosition 值


实战中：
首先 ssh 连上车（车的地址是 192.168.100.2），将 ipv4 地址设为 192.168.100.1，然后将 bash 里的 ROS_MASTER_URI 设置为 192.168.100.2

注意：用 vscode 连车直接是更改的车上的代码，一定要做好代码和参数的备份

下一步我们可以拉曲线看是否卡弹：在本机启动 rosrun plotjugger plotjugger
然后拉出曲线中的 joint state 里的 trigger joint 的 position，正常的情况是阶梯状往下

判断弹链松紧需要先打出一颗弹，然后再看

单独调整 urdf 等参数文件，需要重跑控制器，首先先关闭所有控制器 stop
再启动 roscore
之后启动 mon launch rm_config rm_ecat_hw.launch
以及 mon launch rm_config load_controller.launch
!!勘误：mon launch rm_config load_controllers.launch

下一步可以通过 rqt（本机启动）的 controller manager 来启动
通过 rqt 启动 joint state 和 robot state 控制器，接着手动启动 calibration 控制器，待校准完成后关闭。

那之后可以通过 rqt 手动发射弹丸：
关闭校准控制器，启动 shooter 控制器，然后打开消息发布插件
接着找到/controllers/shooter_controller/command,
如下图所示，
![[images/Pasted image 20260103174957.png]]
需要设置的地方是 rate 100
mode 0 代表关闭，mode 1 代表只启动摩擦论 mode 2 代表开始发射
wheel speed 设置为 700
hz 代表发弹频率，取值在 1-5 之间
打符的 hz 是 1,这是调双发的关键

如果我们通过控制器来进行双发的测试，那么我们通过

mon launch rm_bringup start.launch 会跑起来车上除了视觉之外的所有控制器
视觉是通过 vision_start.launch 启动

设置免密码登陆：ssh-copy-id dynamicx@192.168.100.2
（要先在本机生成密钥）

