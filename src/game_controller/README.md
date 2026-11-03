# game_controller

接收 HSL 2026 `RGme` v20（158 字节），发布 `/robocup/game_controller`。
订阅 Brain 的 `/robocup/game_controller_return`，每秒向最近有效来源 IP 的 UDP 3939
发送 v4（32 字节）；接收数据或反馈过期 2s 后不发送。

完整网络格式与编解码统一在 `game_controller_interface/protocol.hpp`，不直接发送 ROS 消息内存。
20ms 非阻塞轮询与 ROS 回调串行执行；重复、旧包不续期，断流 2s 后重新同步序号。

启动入口为 `launch/launch.py`，默认配置 `config/config.yaml`：3838 端口，白名单默认关闭。
`expected_team_id=0` 时使用 Brain 回报的队号；单独运行接收器时可明确设置现场队号。
本模块只负责网络；比赛权限归 Brain 的 GameState 模块。
