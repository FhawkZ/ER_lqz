# 介绍

本工程是基于python实现的ROS节点程序，实现下列功能：

1. 首先，从Noitom动捕软件获取人体动作数据（BVH格式）

2. 然后，将BVH数据转换成对应机器人关节模型的数据，该步骤称之为Retargeting

   > 目前支持宇树科技的H1模型机器人的retargeting

3. 最后，再把转换后的数据发送给仿真模拟器节点，用于驱动机器人模型。

   > 除了驱动机器人模型，也可以驱动一个火柴人模型（该火柴人跟真人有一样的骨骼结构）

结合仿真模拟器节点的工程，可以实现从Noitom公司提供的动捕软件获取数据并驱动机器人的功能。

下图展示了各个节点的数据流：

![nodes_arch](img/nodes_arch.png)

- **Noitom Mocap Software**：Noitom公司提供专业的动捕软件（如Axis Studio，Hybrid Data Server等），负责提供基于真实人体的动捕数据

  > 请联系info@noitom.com获取

- **mocap_ros_py**：本工程，基于python实现的ROS节点程序，从动捕软件获取数据，retargeting后发送给仿真器。

  > 工程源码：https://github.com/pnmocap/mocap_ros_py.git

- **mocap_ros_cpp**：功能同mocap_ros_py，基于cpp实现。

  > 工程源码：https://github.com/pnmocap/mocap_ros_cpp.git

- **mocap_ros_urdf**：机器人仿真模拟器节点工程，监听来自mocap_ros_py或mocap_ros_urdf的数据，驱动机器人

  > 工程源码：https://github.com/pnmocap/mocap_ros_cpp.git



# 目录结构

| 条目                      | 说明                                                         |
| ------------------------- | ------------------------------------------------------------ |
| lib/                      | 包含了针对各个CPU架构的librobotapi动态库文件，它提供了接口函数，从Axis Studio获取动捕数据，同时也封装了从BVH数据转换为URDF格式的功能。 |
| img/                      | 包含了本说明文档中用到的图片                                 |
| unitree_h1/               | 存放针对宇树科技H1模型的regarting配置文件                    |
| README.md                 | 本说明文档                                                   |
| README_en.md              | 英文版说明文档                                               |
| mocap_robotapi.py         | 封装了librobotapi库，从Axis Studio获取动捕数据，同时提供了BVH数据转换为符合URDF规范的json数据的功能。 |
| requirements_ros1.txt     | 针对ROS1环境，本工程依赖的python库。                         |
| requirements_ros2.txt     | 针对ROS2环境，本工程依赖的python库。                         |
| mocap_to_robot_ros1.py    | 针对ROS1环境，做为一个ros node，调用mocap_robotapi库，转换BVH数据为URDF格式的json数据，publish 数据类型（JointState）到 topic（/joint_states），以驱动机器人模型。 |
| mocap_to_robot_ros2.py    | 针对ROS2环境，功能与mocap_to_robot_ros1.py相同               |
| mocap_to_stickman_ros1.py | 针对ROS1环境，做为一个ros node，调用mocap_robotapi库，将原始BVH数据publish 数据类型（Joy）到 topic（/remoter/action_list），以驱动robot的火柴人模型。 |
| mocap_to_stickman_ros2.py | 针对ROS2环境，功能与mocap_to_stickman_ros1.py相同            |

# 启动步骤

## 前提

- 首先找一台windows PC，安装好Axis Studio软件，并做好配置。
- 其次找一台linux PC，安装好ROS环境。（ROS1或ROS2都可以）
- 根据ROS1还是ROS2环境，启动对应的仿真模拟器节点，详见： [mocap_ros_urdf](https://github.com/pnmocap/mocap_ros_urdf.git)
- 根据要驱动的模型是火柴人还是机器人，启动对应的ROS node

## 下载安装本工程

登录安装了ROS的linux PC，下载本工程，并根据ROS环境安装依赖包。

```
git clone https://github.com/pnmocap/mocap_ros_py.git
cd mocap_ros_py

pip3 install -r requirements_ros1.txt
or
pip3 install -r requirements_ros2.txt
```


## 配置动捕软件Axis Studio

最新一代的动作捕捉软件支持 Perception Neuron Studio 和 Perception Neuron3（Pro）动作捕捉产品。请访问公司官网获取下载地址。

### 启动*Axis Studio*, 打开一个动作数据文件

此时能看到Axis Studio里的3D模型在运动，如下图所示：

   [![img](img/launch_axis_studio.gif)](img/launch_axis_studio.gif)

### 配置 *BVH Broadcasting*

打开设置对话框，选择BVH Broadcasting并使能：

其中Local Address填写运行Axis Studio软件的windows电脑IP，Destination Address填写运行ROS节点的Linux电脑IP。注意需要填写局域网IP，不要填写127.0.0.1这种回环IP。

其余红框部分，需要严格按照图示填写。

![1740031721398](img/bvh_edit.png)



##  启动并配置仿真模拟器节点

这里的ROS URDF模型使用的是宇树科技的H1机器人模型。

请按照文档描述，启动并配置好仿真模拟器节点程序： https://github.com/pnmocap/mocap_ros_urdf.git。



## 启动本ROS节点程序

将对应urdf的retarget.json  copy到工作空间

~~~
cp -r path/to/mocap_ros_py/unitree_h1/retarget.json  mocap_ros_py/retarget.json
~~~

根据需要，可以选择驱动火柴人模型或者机器人模型。

> 两者只能二选一

### 启动火柴人数据节点

```
cd path/to/mocap_ros_py
python mocap_to_stickman_ros1.py
or
python mocap_to_stickman_ros2.py
```

### 启动机器人数据节点

```
cd path/to/mocap_ros_py
python mocap_to_robot_ros1.py
or
python mocap_to_robot_ros2.py
```

