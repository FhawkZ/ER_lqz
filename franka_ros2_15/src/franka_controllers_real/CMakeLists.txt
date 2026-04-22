# 最低 CMake 版本要求（ROS 2 Humble 推荐 3.8+）
cmake_minimum_required(VERSION 3.8)

# 项目名称：必须和 package.xml 中的包名一致
project(serl_franka_controllers)

# 如果是 GCC 或 Clang 编译器，开启严格编译警告（方便调试代码）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================== 1. 查找所有依赖库 =====================
# 找到编译需要的所有 ROS 2 包和第三方库
find_package(ament_cmake REQUIRED)                     # ROS 2 编译工具
find_package(rosidl_default_generators REQUIRED)       # 消息生成器
find_package(controller_interface REQUIRED)            # 控制器接口（核心）
find_package(hardware_interface REQUIRED)              # 硬件接口
find_package(rclcpp REQUIRED)                          # C++ ROS 2 核心
find_package(rclcpp_lifecycle REQUIRED)                # 生命周期节点
find_package(realtime_tools REQUIRED)                  # 实时控制工具
find_package(pluginlib REQUIRED)                       # 插件库（控制器插件）
find_package(geometry_msgs REQUIRED)                    # 几何消息
find_package(sensor_msgs REQUIRED)                     # 传感器消息
find_package(std_msgs REQUIRED)                        # 标准消息
find_package(Eigen3 REQUIRED)                          # 线性代数库（控制器计算）
find_package(franka_semantic_components REQUIRED)      # Franka 官方组件
find_package(franka_msgs REQUIRED)                     # Franka 自定义消息

# 把所有依赖打包成一个变量，方便后面统一使用
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  pluginlib
  geometry_msgs
  sensor_msgs
  std_msgs
  Eigen3
  franka_semantic_components
  franka_msgs
)

# ===================== 2. 生成自定义 ROS 消息 =====================
# 编译项目里的 .msg 文件，生成 C++ 代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZeroJacobian.msg"          # 自定义消息：零空间雅可比
  "msg/SimplifiedFrankaState.msg" # 自定义消息：简化版 Franka 机器人状态
  DEPENDENCIES std_msgs geometry_msgs  # 依赖的标准消息
)

# ===================== 3. 编译核心控制器库 =====================
# 库名称：serl_franka_controllers_lib
# 对应插件文件中的 <library path="serl_franka_controllers_lib">
set(LIBRARY_NAME ${PROJECT_NAME}_lib)

# 创建共享库（.so 文件），源文件是控制器 C++ 代码
add_library(${LIBRARY_NAME} SHARED
  src/cartesian_impedance_controller_real.cpp  # 笛卡尔阻抗控制器（真机版）
)

# 指定头文件路径（编译时去找 include 文件夹）
target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 编译时路径
  $<INSTALL_INTERFACE:include>                            # 安装后系统路径
  ${EIGEN3_INCLUDE_DIRS}  # Eigen 库头文件
)

# 自动给库链接上面声明的所有依赖（ROS 2 专用语法）
ament_target_dependencies(${LIBRARY_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# 链接自定义消息生成的 C++ 代码（让控制器能发布/订阅自定义消息）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${LIBRARY_NAME} "${cpp_typesupport_target}")

# ===================== 4. 注册控制器插件 =====================
# 关键！告诉 ROS 2 插件描述文件在哪里
# 让 controller_manager 能找到这个阻抗控制器
pluginlib_export_plugin_description_file(controller_interface serl_controllers_plugin.xml)

# ===================== 5. 安装规则（编译完把文件复制到系统路径） =====================
# 安装编译好的库文件（.so）到 /install/lib/
install(TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件到 /install/include/
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件（yaml）到 /install/share/serl_franka_controllers/
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 文件到同上路径
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# # 安装 Python 脚本（可执行权限）到 /install/lib/serl_franka_controllers/
# install(PROGRAMS
#   scripts/dual_arm_vr_teleop.py   # 双臂 VR 遥操作
#   scripts/gripper_wrapper.py     # 夹爪包装节点
#   DESTINATION lib/${PROJECT_NAME}
# )

# ===================== 6. 测试配置 =====================
# 开启测试时，自动检查代码规范
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ===================== 7. 导出配置（让其他包能依赖这个包） =====================
ament_export_targets(export_${LIBRARY_NAME})          # 导出库目标
ament_export_include_directories(include)            # 导出头文件路径
ament_export_libraries(${LIBRARY_NAME})              # 导出库文件
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})  # 导出依赖
ament_export_dependencies(rosidl_default_runtime)    # 导出消息运行时
ament_package()  # 结束编译配置，生成 ROS 2 包信息