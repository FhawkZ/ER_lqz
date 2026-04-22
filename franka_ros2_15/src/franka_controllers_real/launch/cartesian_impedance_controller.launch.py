#!/usr/bin/env python3
# 告诉系统这是一个Python3可执行脚本

# ======================== 导入依赖库 ========================
import os    # 操作系统相关工具，处理路径、文件
import sys   # 系统模块，用于添加Python路径
from ament_index_python.packages import get_package_share_directory  # 获取ROS2包的共享目录
from launch import LaunchDescription                                 # Launch文件核心描述类
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction  # Launch动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 加载其他Python Launch文件
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution    # 变量替换、路径拼接
from launch_ros.actions import Node                                       # 启动ROS节点
from launch_ros.substitutions import FindPackageShare                     # 查找ROS2包路径

# ======================== 加载工具脚本 ========================
# 获取 franka_bringup 这个包的共享安装目录
package_share = get_package_share_directory('franka_bringup')

# 拼接工具目录路径：安装目录下的 lib/franka_bringup/utils
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')

# 将工具目录添加到Python系统路径，让Python能找到 launch_utils.py
sys.path.append(os.path.abspath(utils_path))

# 从工具模块导入 load_yaml 函数，用于读取YAML配置文件
from launch_utils import load_yaml

# ======================== 核心函数：生成机器人相关节点 ========================
# OpaqueFunction 会在Launch运行时调用此函数，context是运行上下文
def generate_robot_nodes(context):
    """
    根据YAML配置文件，动态生成所有机械臂的启动项：
    1. 启动 franka 官方基础启动文件
    2. 启动阻抗控制器
    3. 启动夹爪包装节点
    """
    # 从Launch参数中获取 robot_config_file 配置文件路径（运行时解析）
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    
    # 加载YAML配置文件，返回字典格式
    configs = load_yaml(config_file)
    
    # 用于存储所有要启动的节点/Launch项
    nodes = []

    # 遍历YAML里的每一个机械臂配置（支持多臂配置）
    for item_name, config in configs.items():
        # 当前机械臂的命名空间（多臂必备，避免话题冲突）
        namespace = config['namespace']
        
        # ======================== 1. 启动 Franka 基础Launch ========================
        # 包含（启动）franka.launch.py 这个子Launch文件
        nodes.append(
            IncludeLaunchDescription(
                # 指定要包含的Launch文件：franka_bringup/launch/franka.launch.py
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
                    ])
                ),
                # 传递给子Launch文件的所有参数
                launch_arguments={
                    'arm_id': str(config['arm_id']),                          # 机械臂ID
                    'arm_prefix': str(config['arm_prefix']),                  # TF前缀
                    'namespace': str(namespace),                              # 命名空间
                    'urdf_file': str(config['urdf_file']),                    # URDF模型文件
                    'robot_ip': str(config['robot_ip']),                      # 机械臂真实IP
                    'load_gripper': str(config['load_gripper']),              # 是否加载夹爪
                    'use_fake_hardware': str(config['use_fake_hardware']),    # 是否使用仿真硬件
                    'fake_sensor_commands': str(config['fake_sensor_commands']),  # 仿真传感器指令
                    'joint_state_rate': str(config['joint_state_rate']),      # 关节状态发布频率
                    # 控制器配置文件：固定使用 cartesian_impedance_controller_real.yaml
                    'controllers_yaml': str(PathJoinSubstitution([
                        FindPackageShare('serl_franka_controllers'),
                        'config',
                        'cartesian_impedance_controller_real.yaml'
                    ])),
                    # 'controllers_yaml': str(PathJoinSubstitution([
                    #     FindPackageShare('serl_franka_controllers'),
                    #     'config',
                    #     'cartesian_impedance_controller_real.yaml'
                    # ]).perform(context)),
                }.items(),
            )
        )

        # ======================== 2. 启动笛卡尔阻抗控制器 ========================
        # 使用 controller_manager spawner 激活控制器
        nodes.append(
            Node(
                package='controller_manager',        # 包名
                executable='spawner',                # 可执行文件：控制器加载器
                namespace=namespace,                 # 命名空间
                arguments=[
                    'cartesian_impedance_controller',  # 要启动的控制器名称
                    '--controller-manager-timeout', '60',  # 控制器管理器超时时间
                ],
                output='screen',  # 日志输出到终端
            )
        )
    
    # 返回所有生成的节点/Launch项
    return nodes

# ======================== Launch 入口函数 ========================
def generate_launch_description():
    """
    整个Launch文件的入口：
    1. 声明配置文件路径参数
    2. 调用OpaqueFunction动态生成机器人节点
    """
    return LaunchDescription([
        # 声明一个Launch参数：robot_config_file（机器人配置文件路径）
        DeclareLaunchArgument(
            'robot_config_file',
            # 默认路径：franka_bringup/config/franka.config.yaml
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',  # 参数说明
        ),
        
        # 不透明函数：运行时才执行 generate_robot_nodes，实现动态加载配置
        OpaqueFunction(function=generate_robot_nodes),
    ])