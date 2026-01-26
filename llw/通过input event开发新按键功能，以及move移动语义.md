input_event
```cpp
while (ros::ok())
{
  ros::spinOnce();
  manual_control->run();
  loop_rate.sleep();
}
```
在 main.cpp 中，当程序执行到 ros::spinOnce()时，ROS 会检查所有的订阅者（Subscribers）消息队列。如果有新消息到达，ROS 会依次调用这些消息对应的回调函数。

开发按键功能的核心是：
首先每周期会收到一个 dbus data msg，我们对每个要设置的按键定义一个 input event，然后设置不同情况的回调函数：具体通过 `InputEvent` 类进行设置
`InputEvent` 类是一个用于处理布尔输入信号的工具类
其核心功能包括：
    *   `setRising`: 接受一个函数，当状态从 `false` 变为 `true` 时触发。
    *   `setFalling`: 接受一个函数，当状态从 `true` 变为 `false` 时触发。
    *   `setEdge`: 接受一个函数，同时设置上升沿和下降沿的回调函数。
    *   `setActiveHigh`: 接受一个函数，只要状态为 `true`，每次 `update` 都会调用，并传入当前状态持续的时间。
    *   `setActiveLow`: 接受一个函数，只要状态为 `false`，每次 `update` 都会调用，并传入持续时间。
    *   `setDelayTriggered`: 接受一个函数，当检测到指定边沿（上升或下降）时启动一个 `ros::Timer`，在指定的 `duration` 之后执行回调函数。
##  如何开发一个新按键功能 ？
首先可以根据 dbus data msg 的内容确定有什么按键可以用的，然后选一个喜欢的，定义一个InputEvent：
```cpp
InputEvent ctrl_f_event_;
```
然后根据场景不同设置回调方式（按下触发还是松开触发，甚至延迟触发等）
```cpp
ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
```
最后编写对应的回调函数：
```cpp
void EngineerManual::ctrlFPress()
{
  if (root_ == "GET_DOWN_STONE_RIGHT" || root_ == "GET_UP_STONE_RIGHT" || root_ == "RIGHT_EXCHANGE")
  {
    prefix_ = "";
    root_ = "RIGHT_EXCHANGE";
    runStepQueue(root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
    changeSpeedMode(EXCHANGE);
  }
  else if (root_ == "GET_DOWN_STONE_LEFT" || root_ == "GET_UP_STONE_LEFT" || root_ == "LEFT_EXCHANGE")
  {
    prefix_ = "";
    root_ = "LEFT_EXCHANGE";
    runStepQueue(root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
    changeSpeedMode(EXCHANGE);
  }
  else
  {
    prefix_ = "";
    root_ = "EXCHANGE_POS";
    runStepQueue(root_);
    ROS_INFO("%s", (prefix_ + root_).c_str());
    changeSpeedMode(EXCHANGE);
  }
}
```
上面代码摘抄自 engineer_manual.cpp


---
 
# move
移动语义

很神奇的东西

考虑这么一种情况
现在要写一个类 A，其中的成员 a 是类 B 的实例
A 的构造函数接受一个类 B 的对象 b，并将之拷贝给 a
这里出现的拷贝显然不是一个好事情，尤其当类 B 对象 b 是临时的时
过会就销毁了

现在的情况就像造房子，在构造 A 时依着 b 这个房子的样子造了个 a
结果过会又把 b 给拆了
那不如干脆别新建房子，直接把 b 房子的主人变成 a

这就是 move 的用处：换主人
move 声明原主人要死了，即所谓亡值/将亡值

从实际代码上看：
```cpp
boost::function<void()> rising_handler_;
void setRising(boost::function<void()> handler)
{
  rising_handler_ = std::move(handler);
}
```

这里传入的 handler 很显然生命周期就是这个函数段，结束了就死掉了
但如果我们直接写：`rising_handler=handler`
那就出现了拷贝，这不是我们希望看到的

于是 `rising_handler_ = std::move(handler);` 实现了 handler 即将销毁的内容成为了 rising_handler_的所有物，没有额外构造一个 boost::function<void()> 对象

