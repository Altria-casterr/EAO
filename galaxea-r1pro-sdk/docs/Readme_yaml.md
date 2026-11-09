# 根据不同机器人写相关描述文档
```yaml
{
  "robot_id": "r1pro-1.x",  # 机器人-硬件型号.软件型号 
  "backend": "real",        # 引擎
  "firmware_profile": "vendor-2026.08",  # 时间
  "namespace": "r1pro-001",  # 同robot-id
  "endpoint": "ros2://r1pro-001",  # rosbrige端口和ip
  "options": {
    "test_mode": true,
    "defer_commands": false,  # 异步或者同步，同步直接结束，异步running等待
    "stop_confirmed": true
  }
}
```
# 根据不同机器人写backends中的相关适配文件


# 根据不同机器人写message的transport文件

# 根据不同机器人写相关的控制函数


