"""
哨兵系统 (Sentinel) — 总启动文件
GOAI 具身未来赛道 | 产业园区全地形巡逻挑战赛
ROS2 Jazzy / Ubuntu 24.04

启动全部 6 个节点:
  - fast_detector    (YOLOv8 多摄像头检测)
  - vlm_server       (GGUF VLM 推理)
  - arbitrator       (融合仲裁 + 安全兜底)
  - report_generator (结构化报告)
  - task_scheduler   (任务调度)
  - visualizer       (RViz 感知融合可视化)

用法:
  # 无 VLM (纯 YOLO 模式):
  ros2 launch sentinel_bringup sentinel.launch.py

  # 带 VLM:
  ros2 launch sentinel_bringup sentinel.launch.py model_path:=/path/to/llava.gguf

可选参数:
  model_path       VLM GGUF 模型路径 (默认: '')
  vlm_n_ctx        VLM 上下文长度 (默认: 2048)
  multi_camera     是否启用多摄像头 (默认: true)
  detect_interval  检测间隔秒 (默认: 0.5)
  enable_viz       启用可视化 (默认: true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 参数声明 ----
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='VLM GGUF 模型路径'
    )
    vlm_n_ctx_arg = DeclareLaunchArgument(
        'vlm_n_ctx', default_value='2048',
        description='VLM 上下文长度'
    )
    multi_camera_arg = DeclareLaunchArgument(
        'multi_camera', default_value='true',
        description='启用 S10 4路全向相机'
    )
    detect_interval_arg = DeclareLaunchArgument(
        'detect_interval', default_value='0.5',
        description='YOLO 检测间隔 (秒)'
    )
    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz', default_value='true',
        description='启用 RViz 感知融合可视化'
    )

    # ---- 节点 ----
    fast_detector = Node(
        package='sentinel_detector',
        executable='fast_detector',
        name='fast_detector',
        output='screen',
        parameters=[{
            'multi_camera': LaunchConfiguration('multi_camera'),
            'detect_interval': LaunchConfiguration('detect_interval'),
        }],
    )

    vlm_server = Node(
        package='sentinel_vlm',
        executable='vlm_server',
        name='vlm_server',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'n_ctx': LaunchConfiguration('vlm_n_ctx'),
        }],
    )

    arbitrator = Node(
        package='sentinel_arbitrator',
        executable='arbitrator',
        name='arbitrator',
        output='screen',
        parameters=[{
            'vlm_model_path': LaunchConfiguration('model_path'),
            'vlm_n_ctx': LaunchConfiguration('vlm_n_ctx'),
        }],
    )

    report_generator = Node(
        package='sentinel_report',
        executable='report_generator',
        name='report_generator',
        output='screen',
    )

    task_scheduler = Node(
        package='sentinel_mission',
        executable='task_scheduler',
        name='task_scheduler',
        output='screen',
    )

    visualizer = Node(
        package='sentinel_viz',
        executable='visualizer',
        name='sentinel_viz',
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        vlm_n_ctx_arg,
        multi_camera_arg,
        detect_interval_arg,
        enable_viz_arg,
        LogInfo(msg=['=== Sentinel 哨兵系统 (GOAI 比赛版) ===']),
        LogInfo(msg=['平台: 山猫 S10 | 场地: 杭州云谷中心']),
        LogInfo(msg=['ROS2: Jazzy | Ubuntu: 24.04']),
        fast_detector,
        vlm_server,
        arbitrator,
        report_generator,
        task_scheduler,
        visualizer,
        LogInfo(msg=['Sentinel 6 节点已启动, 等待摄像头数据...']),
        LogInfo(msg=['可视化: RViz -> Add MarkerArray -> /sentinel/viz/markers']),
    ])
