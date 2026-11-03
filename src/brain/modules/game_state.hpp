#pragma once
#include "common/input_types.hpp"
#include "common/module.h"
#include "modules/ball_state.hpp"
#include "modules/self_state.hpp"
#include "modules/field_provider.hpp"
#include <stdexcept>

namespace brain {
// 权限与原始裁判数据分开；当前只有直射式 Dribble，不把它当受限传球。
struct GameState {
    int team = 8, player = 1, state = gc::Initial, phase = gc::Normal, penalty = 0;
    unsigned goalkeeper = 0;
    bool connected = false, canMove = false, canKick = false;
    gc::Packet packet;
    string reason = "waiting_gc";
    string detail() const { return reason; }
};
class GameStateModule : public Module {
public:
    GameStateModule(const Sample<gc::Packet>& input, const BallState& ball,
                    const SelfState& self, const FieldModel& field, const double& now)
        : input_(input), ball_(ball), self_(self), field_(field), now_(now) {}
    GameState output;
    ModuleInfo info() const override {
        return {"GameState", {"GameControllerData","BallState","SelfState","FieldModel","FrameTime"}, {"GameState"}};
    }
    void configure(rclcpp::Node& node) override {
        output.team=node.declare_parameter<int>("team_id",8);
        output.player=node.declare_parameter<int>("player_id",1);
        if(output.team<1 || output.team>255 || output.player<1 || output.player>20)
            throw invalid_argument("team_id must be 1..255; player_id must be 1..20");
    }
    void run() override { update(); }
    void update() {
        output.canMove=false; output.canKick=false; output.connected=false;
        if(!input_.valid() || now_<input_.updatedAt || now_-input_.updatedAt>2.5) {
            output.reason="gc_timeout"; return;
        }
        const gc::Packet& p=input_.value;
        if(!gc::valid(p) || !gc::containsTeam(p,output.team)) { output.reason="invalid_gc"; return; }
        output.connected=true; output.packet=p; output.state=p.state; output.phase=p.phase;
        const gc::Team& own=p.teams[p.teams[0].number==output.team ? 0 : 1];
        output.penalty=own.players[output.player-1].penalty; output.goalkeeper=own.goalkeeper;
        const unsigned expected=field_.dimensions.length==9 ? 0 : field_.dimensions.length==14 ? 1 : 2;
        if(p.competitionType!=expected) { output.reason="field_type_mismatch"; return; }

        // READY/SET 是新一轮开球准备；旧定位球及触球记忆不能跨半场或重置保留。
        const bool newSetup=p.state!=gc::Playing && previousState_!=p.state;
        const bool newEpisode=newSetup || p.firstHalf!=previousHalf_ || p.phase!=previousPhase_ ||
            (p.setPlay!=gc::None && (p.setPlay!=previousSetPlay_ || p.kickingTeam!=owner_)) ||
            (restart_!=gc::None && p.kickingTeam!=gc::noTeam && p.kickingTeam!=owner_);
        if(newEpisode) {
            restart_=p.setPlay;
            if(p.phase==gc::PenaltyShootout) restart_=gc::PenaltyKick;
            if(p.phase!=gc::PenaltyShootout && restart_==gc::None && p.kickingTeam!=gc::noTeam && p.state!=gc::Finished) restart_=kickoff;
            owner_=p.kickingTeam;
            hasOrigin_=false; moved_=false; observation_=0; episodeAt_=now_;
        }
        // 首包可能直接是 PLAY；不猜测它已经是自由球。
        if(previousState_<0 && p.state==gc::Playing) {
            restart_=p.setPlay;
            if(p.phase==gc::PenaltyShootout) restart_=gc::PenaltyKick;
            if(p.phase!=gc::PenaltyShootout && restart_==gc::None && p.kickingTeam!=gc::noTeam) restart_=kickoff;
            owner_=p.kickingTeam; episodeAt_=now_;
        }
        if(p.setPlay==gc::None && p.kickingTeam==gc::noTeam && p.phase!=gc::PenaltyShootout) {
            restart_=gc::None; owner_=gc::noTeam;
        }
        if(previousStopped_ && !p.stopped) { hasOrigin_=false; moved_=false; episodeAt_=now_; }
        previousState_=p.state; previousSetPlay_=p.setPlay; previousHalf_=p.firstHalf;
        previousPhase_=p.phase; previousStopped_=p.stopped!=0;

        if(p.stopped) { output.reason="stopped"; return; }
        if(p.phase==gc::Timeout) { output.reason="game_timeout"; return; }
        if(p.state!=gc::Playing) {
            const char* names[]={"initial","ready_hold","set","playing","finished"};
            output.reason=names[p.state]; return;
        }
        if(output.penalty) { output.reason="penalized"; return; }
        if(self_.recovery.valid() && self_.recovery.value.state!=0) {
            hasOrigin_=false; moved_=false; episodeAt_=now_;
            output.reason="recovering"; return;
        }
        // 仅新的真实视觉帧可证明球移动；用里程计消除机器人自身位移。
        if(ball_.visible && ball_.seenAt>=episodeAt_ && now_-ball_.seenAt<=.3 &&
           ball_.detectionRevision!=observation_ && self_.odometry.valid() &&
           now_>=self_.odometry.updatedAt && now_-self_.odometry.updatedAt<=.3) {
            observation_=ball_.detectionRevision;
            const Pose2D point=compose(self_.odometry.value,ball_.ball.relative);
            if(!hasOrigin_) { origin_=point; hasOrigin_=true; }
            else if(hypot(point.x-origin_.x,point.y-origin_.y)>.2) moved_=true;
        }
        if(restart_!=gc::None) {
            if(owner_==gc::noTeam) { output.reason="restart_owner_missing"; return; }
            if(owner_!=static_cast<unsigned>(output.team)) {
                if(!moved_) { output.reason="opponent_restart_wait_ball"; return; }
                // 对方已把球投入比赛；后续 setPlay 残留不重新布置同一轮等待。
            } else {
                if(restart_==kickoff || restart_==gc::IndirectFreeKick || restart_==gc::ThrowIn) {
                    output.reason="restart_requires_restricted_pass_or_team_touch"; return;
                }
                if(moved_) { output.reason="own_restart_second_touch_unverified"; return; }
            }
        }
        // 点球决胜只放行本队未受罚球员；执行者由裁判上场名单限定，对方执行时保持站立。
        if(p.phase==gc::PenaltyShootout && p.kickingTeam!=output.team) {
            output.reason="opponent_penalty_shootout"; return;
        }
        output.canMove=true; output.canKick=true; output.reason="playing";
    }
private:
    static const unsigned kickoff=7;
    const Sample<gc::Packet>& input_;
    const BallState& ball_;
    const SelfState& self_;
    const FieldModel& field_;
    const double& now_;
    int previousState_=-1;
    unsigned restart_=gc::None, owner_=gc::noTeam, previousSetPlay_=gc::None, previousHalf_=0, previousPhase_=0;
    bool previousStopped_=false, hasOrigin_=false, moved_=false;
    Pose2D origin_;
    std::uint64_t observation_=0;
    double episodeAt_=0;
};
}
