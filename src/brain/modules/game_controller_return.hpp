#pragma once
#include "modules/game_state.hpp"
#include "modules/localization.hpp"
#include <game_controller_interface/msg/game_control_return_data.hpp>
#include <algorithm>

namespace brain {
// 只组装本机反馈；网络目标、有效接收期限和 1Hz 发送由裁判接收节点负责。
class GameControllerReturnModule : public Module {
public:
    GameControllerReturnModule(const GameState& game, const SelfState& self, const BallState& ball,
                               const LocalizationState& localization, const double& now)
        : game_(game), self_(self), ball_(ball), localization_(localization), now_(now) {}
    gc::ReturnData output;
    ModuleInfo info() const override {
        return {"GameControllerReturn", {"GameState","SelfState","BallState","LocalizationState","FrameTime"}, {"GameControllerReturn"}};
    }
    void configure(rclcpp::Node& node) override {
        publisher_=node.create_publisher<game_controller_interface::msg::GameControlReturnData>("/robocup/game_controller_return",1);
    }
    gc::ReturnData feedback() const {
        gc::ReturnData r;
        r.team=static_cast<std::uint8_t>(game_.team); r.player=static_cast<std::uint8_t>(game_.player);
        r.fallen=!(self_.recovery.valid() && now_>=self_.recovery.updatedAt &&
                   now_-self_.recovery.updatedAt<1 && self_.recovery.value.state==0);
        const Pose2D& pose=localization_.poseToField;
        if(localization_.valid && isfinite(pose.x) && isfinite(pose.y) && isfinite(pose.yaw)) {
            r.pose[0]=static_cast<float>(pose.x*1000); r.pose[1]=static_cast<float>(pose.y*1000);
            r.pose[2]=static_cast<float>(pose.yaw);
        }
        if(ball_.seenAt>0 && now_>=ball_.seenAt && isfinite(ball_.ball.relative.x) && isfinite(ball_.ball.relative.y)) {
            r.ballAge=static_cast<float>(min(3600.0,now_-ball_.seenAt));
            r.ball[0]=static_cast<float>(ball_.ball.relative.x*1000); r.ball[1]=static_cast<float>(ball_.ball.relative.y*1000);
        }
        return r;
    }
    void run() override {
        if(now_-sentAt_<1 && now_>=sentAt_) return;
        output=feedback();
        const gc::ReturnData& r=output;
        game_controller_interface::msg::GameControlReturnData msg;
        msg.sampled_at=now_;
        msg.team_num=r.team; msg.player_num=r.player; msg.fallen=r.fallen;
        for(unsigned i=0;i<3;++i) msg.pose[i]=r.pose[i];
        msg.ball_age=r.ballAge;
        for(unsigned i=0;i<2;++i) msg.ball[i]=r.ball[i];
        publisher_->publish(msg); sentAt_=now_;
    }
private:
    const GameState& game_;
    const SelfState& self_;
    const BallState& ball_;
    const LocalizationState& localization_;
    const double& now_;
    double sentAt_=-1;
    rclcpp::Publisher<game_controller_interface::msg::GameControlReturnData>::SharedPtr publisher_;
};
}
