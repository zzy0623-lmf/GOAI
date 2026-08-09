"""
哨兵系统 (Sentinel) — 总启动文件
GOAI 具身未来赛道 | 产业园区全地形巡逻挑战赛

启动全部 5 个节点:
  - fast_detector    (YOLOv8 多摄像头检测)
  - vlm_server       (GGUF VLM 推理)
  - arbitrator       (融合仲裁 + 安全兜底)
  - report_generator (结构化报告)
  - task_scheduler   (任务调度)

用法:
  ros2 launch sentinel_bringup sentinel.launch.py

可选参数:
  model_path       VLM GGUF 模型路径 (默认: '')
  vlm_n_ctx        VLM 上下文长度 (默认: 2048)
  multi_camera     是否启用多摄像头 (默认: true)
  detect_interval  检测间隔秒 (默认: 0.5)
  single_camera    单相机兼容模式 (默认: false)
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
    single_camera_arg = DeclareLaunchArgument(
        'single_camera', default_value='false',
        description='单相机兼容模式 (覆写 multi_camera)'
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
        condition=None,  # 始终启动
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

    return LaunchDescription([
        model_path_arg,
        vlm_n_ctx_arg,
        multi_camera_arg,
        detect_interval_arg,
        single_camera_arg,
        LogInfo(msg=['启动 Sentinel 哨兵系统 (GOAI 比赛版)']),
        LogInfo(msg=['平台: 山猫 S10 | 场地: 杭州云谷中心']),
        fast_detector,
        vlm_server,
        arbitrator,
        report_generator,
        task_scheduler,
        LogInfo(msg=['Sentinel 全节点已启动, 等待摄像头数据...']),
    ])
