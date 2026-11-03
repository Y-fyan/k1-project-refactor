# HSL 2026 v20。UDP wire 在 protocol.hpp 中显式解码。
char[4] header
uint8 version
uint8 packet_number
uint8 players_per_team
uint8 competition_type
bool stopped
uint8 game_phase
uint8 state
uint8 set_play
uint8 first_half
uint8 kicking_team
int16 secs_remaining
int16 secondary_time
TeamInfo[2] teams

# 同机单调时钟接收时间，供 Brain 判定网络数据时效；不属于 UDP 报文。
float64 received_at
