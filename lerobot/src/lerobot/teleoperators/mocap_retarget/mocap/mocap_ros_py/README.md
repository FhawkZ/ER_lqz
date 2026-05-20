# Introduction

This project is a ROS node program implemented in Python, which achieves the following functions:

1. First, obtain human motion data (in BVH format) from the Noitom Mocap Software.

2. Then, convert the BVH data into data corresponding to the robot joint model. This step is called Retargeting.

   > Currently, retargeting for the H1 model robot of Unitree Technology is supported.

3. Finally, send the converted data to the simulation emulator node to drive the robot model.

   > In addition to driving the robot model, it can also drive a TF model (stickman has the same skeletal structure as a real human).

When combined with the project of the simulation emulator node, it can achieve the function of obtaining data from the Noitom Mocap Software and using it to drive the robot.

The following figure shows the data flow of each node:

![nodes_arch](img/nodes_arch.png)

- **Noitom Mocap Software**: Noitom Company provides professional motion capture software (such as Axis Studio, Hybrid Data Server, etc.), which is responsible for providing motion capture data based on real human bodies.

  > please contact info@noitom.com

- **mocap_ros_py**:  This project. A ROS node program implemented in Python. It retrieves data from the Noitom Mocap Software. After retargeting, it sends the data to the simulator to drive the robot model. Alternatively, it can directly send the BVH data to drive the TF model(stickman). 

  > source code: https://github.com/pnmocap/mocap_ros_py.git

- **mocap_ros_cpp**: It has the same function as mocap_ros_py, but it is implemented in C++. 

  > source code: https://github.com/pnmocap/mocap_ros_cpp.git

- **mocap_ros_urdf**: A robot simulation emulator, listens to the data from mocap_ros_py or mocap_ros_urdf and drives the robot.

  > source code:  https://github.com/pnmocap/mocap_ros_urdf.git



# File Structure

| Item                      | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| lib/                      | It includes the `librobotapi` dynamic library files for various CPU architectures. This library provides interface functions to obtain motion capture data from Axis Studio and also encapsulates the functionality of converting BVH data to the URDF format. |
| img/                      | It contains the images used in this instruction document.    |
| unitree_h1/               | It stores the retargeting configuration files for the Unitree Technology H1 model. |
| README.md                 | Readme in Chinese                                            |
| README_en.md              | Readme in English                                            |
| mocap_robotapi.py         | It encapsulates the `librobotapi` library to obtain motion capture data from Axis Studio. Meanwhile, it provides the function of converting BVH data into JSON data that complies with the URDF specification.分享 |
| requirements_ros1.txt     | For the ROS 1 environment, the Python libraries that this project depends on.分享 |
| requirements_ros2.txt     | For the ROS 2 environment, the Python libraries that this project depends on.分享 |
| mocap_to_robot_ros1.py    | In the ROS 1 environment, as a ROS node, this project calls the `mocap_robotapi` library to convert BVH data into JSON data in the URDF format. Then it publishes the data of type `JointState` to the topic `/joint_states` to drive the robot model.分享 |
| mocap_to_robot_ros2.py    | In the ROS 2 environment, the functionality is the same as that of `mocap_to_robot_ros1.py`. |
| mocap_to_stickman_ros1.py | In the ROS 1 environment, as a ROS node, it calls the `mocap_robotapi` library to publish the original BVH data of the data type `Joy` to the topic `/remoter/action_list` to drive the stick - figure model of the robot. |
| mocap_to_stickman_ros2.py | In the ROS 2 environment, the functionality is identical to that of the `mocap_to_stickman_ros1.py` script. |

# Launch Steps

## Prerequisites

- First, find a Windows PC, install the Axis Studio software, and complete the configuration.
- Second, find a Linux PC and install the ROS environment. (Either ROS 1 or ROS 2 is acceptable.)
- Depending on whether it is a ROS 1 or ROS 2 environment, start the corresponding simulation emulator node. For details, refer to: [mocap_ros_urdf](https://github.com/pnmocap/mocap_ros_urdf.git).
- According to whether the model to be driven is a TF or a robotmodel, start the corresponding ROS node.

## Download and Install This Project

Log in to the Linux PC with ROS installed, download this project, and install the dependent packages according to the ROS environment.

```
git clone https://github.com/pnmocap/mocap_ros_py.git
cd mocap_ros_py

pip3 install -r requirements_ros1.txt
or
pip3 install -r requirements_ros2.txt
```


## Configure the Motion Capture Software: Axis Studio

The latest generation of motion capture software supports the Perception Neuron Studio and Perception Neuron3 (Pro) motion capture products. Please refer to the official website of Noitom Company for more information.

### Start *Axis Studio* and Open a Motion Data File

At this point, you can see the 3D model in Axis Studio moving, as shown in the following figure:

   [![img](img/launch_axis_studio.gif)](img/launch_axis_studio.gif)

### Configure *BVH Broadcasting*

Open the settings dialog box, select BVH Broadcasting and enable it:

Fill in the "Local Address" with the IP address of the Windows computer running the Axis Studio software, and fill in the "Destination Address" with the IP address of the Linux computer running the ROS node. Note that you need to fill in the local area network IP address, not a loopback IP like 127.0.0.1.

For the rest of the parts within the red frame, you need to fill them in strictly as shown in the figure.

![1740031721398](img/bvh_edit.png)



## Start and Configure the Simulation Emulator Node

The ROS URDF model used here is the H1 robot model from Unitree Technology.

Please follow the description in the documentation to start and configure the simulation emulator node program: https://github.com/pnmocap/mocap_ros_urdf.git.



## Start This ROS Node Program

Copy the corresponding `retarget.json` file of the URDF to the workspace.

~~~
cp -r path/to/mocap_ros_py/unitree_h1/retarget.json  mocap_ros_py/retarget.json
~~~

You can choose to drive either the TF model or the robot model as needed.

> You can only choose one of the two.

### Launch TF Data Node

```
cd path/to/mocap_ros_py
python mocap_to_stickman_ros1.py
or
python mocap_to_stickman_ros2.py
```

### Launch RobotModel Data Node

```
cd path/to/mocap_ros_py
python mocap_to_robot_ros1.py
or
python mocap_to_robot_ros2.py
```

