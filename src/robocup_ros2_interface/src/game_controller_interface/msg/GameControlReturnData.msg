# Brain 提供状态，裁判接收节点负责每秒向有效来源发送 v4 回包。
uint8 player_num
uint8 team_num
uint8 fallen
float32[3] pose
float32 ball_age
float32[2] ball

# 同机单调时钟采样时间，避免 ROS 排队把旧回报刷新为新状态。
float64 sampled_at
