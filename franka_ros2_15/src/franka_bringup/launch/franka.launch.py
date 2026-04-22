# Copyright (c) 2025 Franka Robotics GmbH
#
# 本代码受 Apache 2.0 开源协议保护
# 你可以在遵守协议的前提下自由使用、修改、分发
# 协议详情：http://www.apache.org/licenses/LICENSE-2.0

############################################################################
# 启动参数说明：
# arm_id: 机械臂型号ID（默认空）
# arm_prefix: 机械臂话题前缀（默认空）
# namespace: 机器人命名空间（默认空，多臂必须用）
# urdf_file: URDF 文件路径（相对 franka_description/robots，默认 fr3/fr3.urdf.xacro）
# robot_ip: 机械臂控制柜 IP 地址（默认 172.16.0.3）
# load_gripper: 是否加载 Franka 原装夹爪（默认 false）
# use_fake_hardware: 是否使用仿真硬件（无真机时用，默认 false）
# fake_sensor_commands: 仿真传感器指令（默认 false）
# joint_state_rate: 关节状态发布频率 Hz（默认 30）
#
# 本文件作用：启动 Franka 核心组件
# 包括：机器人状态发布、ros2_control 控制节点、关节状态广播、机器人状态广播、夹爪驱动
# 支持命名空间（多臂隔离）、仿真/真机切换
#
# 使用示例：
# ros2 launch franka_bringup franka.launch.py arm_id:=fr3 namespace:=franka1 robot_ip:=172.16.0.3
#
# 推荐用法：把参数写进 YAML 配置文件（franka.config.yaml），方便多臂管理
# 命名空间作用：给每个机械臂话题加前缀，例如 /franka1/joint_states，避免多臂冲突
############################################################################

# 导入 xacro：用于解析 URDF xacro 文件
import xacro

# ROS 2 Launch 基础模块
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 条件判断：满足/不满足条件时才执行
from launch.conditions import UnlessCondition, IfCondition

# 配置读取、路径拼接
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# 启动节点、查找包路径
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ===================== 核心函数：生成机器人所有节点 =====================
# OpaqueFunction：运行时才执行，能动态读取参数、处理文件
def generate_robot_nodes(context):
    """
    生成 Franka 机械臂所有核心节点：
    1. 解析 URDF（xacro）
    2. 启动 robot_state_publisher
    3. 启动 ros2_control_node（控制器核心）
    4. 启动 joint_state_publisher
    5. 启动 2 个广播器（关节状态 + 机器人状态）
    6. 可选启动夹爪驱动
    """
    # ===================== 读取启动参数 =====================
    # 是否加载夹爪（转布尔值）
    load_gripper_launch_configuration = LaunchConfiguration('load_gripper').perform(context)
    load_gripper = load_gripper_launch_configuration.lower() == 'true'

    # 拼接 URDF 文件完整路径
    urdf_path = PathJoinSubstitution([
        FindPackageShare('franka_description'), 'robots', LaunchConfiguration('urdf_file')
    ]).perform(context)

    # ===================== 用 xacro 解析生成机器人描述 =====================
    # 处理 xacro，传入参数生成最终 URDF 字符串
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',                  # 启用 ros2_control
            'arm_id': LaunchConfiguration('arm_id').perform(context),
            'arm_prefix': LaunchConfiguration('arm_prefix').perform(context),
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'hand': load_gripper_launch_configuration,  # 是否带夹爪
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands').perform(context),
        }
    ).toprettyxml(indent='  ')  # 格式化输出 XML

    # ===================== 其他配置 =====================
    namespace = LaunchConfiguration('namespace').perform(context)  # 命名空间

    # 控制器配置文件路径
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('franka_bringup'), 'config', "controllers.yaml"
    ]).perform(context)

    # 关节状态来源：机械臂 + 夹爪
    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']

    # 关节状态发布频率
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))

    # ===================== 开始创建所有节点 =====================
    nodes = [
        # -------------------- 1. 机器人状态发布器 --------------------
        # 读取 URDF，发布 TF 和 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # -------------------- 2. ros2_control 核心控制节点 --------------------
        # 真正和硬件（真机/仿真）通信的节点
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,                # 控制器配置
                {'robot_description': robot_description},
                {'load_gripper': load_gripper}    # 是否加载夹爪
            ],
            remappings=[('joint_states', joint_state_publisher_sources[0])],  # 话题重映射
            output='screen',
            on_exit=Shutdown(),  # 此节点退出则关闭整个启动系统
        ),

        # -------------------- 3. 关节状态聚合器 --------------------
        # 合并机械臂 + 夹爪的关节状态
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{
                'source_list': joint_state_publisher_sources,
                'rate': joint_state_rate,
                'use_robot_description': False,
            }],
            output='screen',
        ),

        # -------------------- 4. 启动关节状态广播器 --------------------
        # 必启动：发布 /joint_states
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # -------------------- 5. 启动机器人状态广播器 --------------------
        # 真机才启动：发布 Franka 内部状态（力、力矩、机器人状态）
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['franka_robot_state_broadcaster'],
            parameters=[{'arm_id': LaunchConfiguration('arm_id').perform(context)}],
            condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),  # 非仿真才运行
            output='screen',
        ),

        # -------------------- 6. 启动夹爪驱动（可选） --------------------
        # 只有 load_gripper=true 才启动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={
                'namespace': namespace,
                'robot_ip': LaunchConfiguration('robot_ip').perform(context),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            }.items(),
            condition=IfCondition(LaunchConfiguration('load_gripper')),
        ),
    ]

    return nodes

# ===================== Launch 文件入口 =====================
def generate_launch_description():
    """
    启动文件入口：
    1. 声明所有可配置参数
    2. 调用 OpaqueFunction 运行节点生成函数
    """
    # 声明所有启动参数（可在命令行覆盖）
    launch_args = [
        DeclareLaunchArgument('arm_id',
                              default_value='',
                              description='机械臂型号ID'),
        DeclareLaunchArgument('arm_prefix',
                              default_value='',
                              description='机械臂话题前缀'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='机器人命名空间（多臂必备）'),
        DeclareLaunchArgument('urdf_file',
                              default_value='fr3/fr3.urdf.xacro',
                              description='URDF xacro 文件路径'),
        DeclareLaunchArgument('robot_ip',
                              default_value='172.16.0.1',
                              description='机械臂控制柜 IP'),
        DeclareLaunchArgument('load_gripper',
                              default_value='false',
                              description='是否加载 Franka 夹爪'),
        DeclareLaunchArgument('use_fake_hardware',
                              default_value='false',
                              description='是否使用仿真硬件'),
        DeclareLaunchArgument('fake_sensor_commands',
                              default_value='false',
                              description='仿真传感器指令'),
        DeclareLaunchArgument('joint_state_rate',
                              default_value='30',
                              description='关节状态发布频率 Hz'),
    ]

    # 返回：参数 + 动态生成的节点
    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])