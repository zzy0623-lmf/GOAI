# 哨兵异常事件消息定义
# GOAI 产业园区全地形巡逻挑战赛

string event_id             # 唯一事件ID

uint8 INFO=0                # 严重级别常量定义
uint8 WARNING=1
uint8 CRITICAL=2
uint8 severity              # 0=INFO, 1=WARNING, 2=CRITICAL

string anomaly_type         # 异常类型: fire, intrusion, equipment_damage 等
float32 confidence          # 置信度 0.0~1.0
string description          # 异常描述

float64 lat                 # 纬度
float64 lng                 # 经度
uint32 waypoint_id          # 巡检点ID
string detection_source     # 检测来源: yolo / vlm / fusion
string image_path           # 关联图像路径
