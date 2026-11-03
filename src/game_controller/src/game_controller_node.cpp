#include "game_controller_node.h"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

GameControllerNode::GameControllerNode(std::string name) : Node(name) {
    port_=declare_parameter<int>("port",gc::dataPort);
    expectedTeam_=declare_parameter<int>("expected_team_id",0);
    whitelist_=declare_parameter<bool>("enable_ip_white_list",false);
    allowedIPs_=declare_parameter<std::vector<std::string> >("ip_white_list",std::vector<std::string>{});
    if(port_<1 || port_>65535 || expectedTeam_<0 || expectedTeam_>255)
        throw std::invalid_argument("invalid GC port or expected_team_id");
    publisher_=create_publisher<game_controller_interface::msg::GameControlData>("/robocup/game_controller",10);
    feedbackSub_=create_subscription<game_controller_interface::msg::GameControlReturnData>(
        "/robocup/game_controller_return",1,[this](const game_controller_interface::msg::GameControlReturnData::SharedPtr msg) {
            gc::ReturnData data;
            data.team=msg->team_num; data.player=msg->player_num; data.fallen=msg->fallen;
            for(unsigned i=0;i<3;++i) data.pose[i]=msg->pose[i];
            data.ballAge=msg->ball_age;
            for(unsigned i=0;i<2;++i) data.ball[i]=msg->ball[i];
            if(!gc::valid(data) || (expectedTeam_ && data.team!=expectedTeam_) ||
               !std::isfinite(msg->sampled_at) || msg->sampled_at<0 || msg->sampled_at>now()) return;
            if(packetAt_ && !gc::containsTeam(packet_,data.team)) { packetAt_=0; sequence_.reset(); }
            feedback_=data; feedbackAt_=msg->sampled_at;
        });
}
GameControllerNode::~GameControllerNode() { if(socket_>=0) close(socket_); }
double GameControllerNode::now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void GameControllerNode::init() {
    socket_=socket(AF_INET,SOCK_DGRAM,0);
    if(socket_<0) throw std::runtime_error(std::strerror(errno));
    const int flags=fcntl(socket_,F_GETFL,0);
    if(flags<0 || fcntl(socket_,F_SETFL,flags|O_NONBLOCK)<0) throw std::runtime_error(std::strerror(errno));
    sockaddr_in local{}; local.sin_family=AF_INET; local.sin_port=htons(static_cast<std::uint16_t>(port_));
    local.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(socket_,reinterpret_cast<sockaddr*>(&local),sizeof(local))<0) throw std::runtime_error(std::strerror(errno));
    // 收发和 ROS 回调由同一个执行器串行处理，无共享线程状态。
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){poll();});
}
void GameControllerNode::poll() {
    std::array<std::uint8_t,512> bytes{};
    // 限制单次回调的处理量，异常流量不能饿死 ROS 回包和状态订阅。
    for(unsigned i=0;i<64;++i) {
        sockaddr_in sender{}; socklen_t length=sizeof(sender);
        const ssize_t size=recvfrom(socket_,bytes.data(),bytes.size(),0,reinterpret_cast<sockaddr*>(&sender),&length);
        if(size<0) {
            if(errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR)
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"GC receive: %s",std::strerror(errno));
            break;
        }
        if(whitelist_ && std::find(allowedIPs_.begin(),allowedIPs_.end(),inet_ntoa(sender.sin_addr))==allowedIPs_.end()) continue;
        gc::Packet packet;
        if(!gc::decode(bytes.data(),static_cast<std::size_t>(size),packet)) continue;
        const unsigned team=expectedTeam_ ? static_cast<unsigned>(expectedTeam_) : feedback_.team;
        if(team && !gc::containsTeam(packet,team)) continue;
        const double time=now();
        if(packetAt_ && time-packetAt_<=2 && source_.sin_addr.s_addr!=sender.sin_addr.s_addr) continue;
        if(source_.sin_addr.s_addr!=sender.sin_addr.s_addr) sequence_.reset();
        if(!sequence_.accept(packet.number,time)) continue;
        source_=sender; source_.sin_port=htons(gc::returnPort);
        packet_=packet; packetAt_=time; publish(packet);
    }
    const double time=now();
    if(packetAt_ && feedbackAt_ && time-packetAt_<2 && time-feedbackAt_<2 &&
       time-sentAt_>=1 && gc::containsTeam(packet_,feedback_.team)) {
        sentAt_=time;
        gc::ReturnData outgoing=feedback_;
        if(outgoing.ballAge>=0)
            outgoing.ballAge=static_cast<float>(std::min(3600.0,outgoing.ballAge+time-feedbackAt_));
        const auto bytes=gc::encode(outgoing);
        const ssize_t sent=sendto(socket_,bytes.data(),bytes.size(),0,reinterpret_cast<sockaddr*>(&source_),sizeof(source_));
        if(sent!=static_cast<ssize_t>(bytes.size())) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"GC return send failed: %s",std::strerror(errno));
    }
}
void GameControllerNode::publish(const gc::Packet& data) {
    game_controller_interface::msg::GameControlData msg;
    for(unsigned i=0;i<4;++i) msg.header[i]=data.header[i];
    msg.received_at=packetAt_;
    msg.version=data.version; msg.packet_number=data.number; msg.players_per_team=data.playersPerTeam;
    msg.competition_type=data.competitionType; msg.stopped=data.stopped!=0; msg.game_phase=data.phase;
    msg.state=data.state; msg.set_play=data.setPlay; msg.first_half=data.firstHalf; msg.kicking_team=data.kickingTeam;
    msg.secs_remaining=data.secsRemaining; msg.secondary_time=data.secondaryTime;
    for(unsigned i=0;i<2;++i) {
        const gc::Team& t=data.teams[i]; auto& target=msg.teams[i];
        target.team_number=t.number; target.field_player_colour=t.fieldColour; target.goalkeeper_colour=t.keeperColour;
        target.goalkeeper=t.goalkeeper; target.score=t.score; target.penalty_shot=t.penaltyShot;
        target.single_shots=t.singleShots; target.message_budget=t.messageBudget;
        for(unsigned j=0;j<gc::maxPlayers;++j) {
            target.players[j].penalty=t.players[j].penalty;
            target.players[j].secs_till_unpenalised=t.players[j].secsTillUnpenalised;
            target.players[j].cautions=t.players[j].cautions;
        }
    }
    publisher_->publish(msg);
}
