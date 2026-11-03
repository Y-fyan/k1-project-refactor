#include "brain.h"
#include <chrono>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <booster_interface/msg/odometer.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <vision_interface/msg/detections.hpp>
#include <vision_interface/msg/line_segments.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <booster_interface/msg/raw_bytes_msg.hpp>

using namespace std;

static double inputTime() {
    return chrono::duration<double>(
        chrono::steady_clock::now().time_since_epoch()).count();
}

// ROS 类型在入口转换为普通数据，算法可以脱离 ROS 使用同一份关节信息。
static vector<brain::JointState> convertJoints(
    const vector<booster_interface::msg::MotorState>& motors) {
    vector<brain::JointState> joints(motors.size());
    for(size_t i = 0; i < motors.size(); ++i) {
        const auto& source = motors[i];
        brain::JointState& target = joints[i];
        target.mode = source.mode;
        target.q = source.q;
        target.dq = source.dq;
        target.ddq = source.ddq;
        target.tauEst = source.tau_est;
        target.temperature = source.temperature;
        target.lost = source.lost;
        target.reserve = source.reserve;
    }
    return joints;
}

class BrainNode : public rclcpp::Node {
public:
    BrainNode()
        : Node("brain_node"),
          brain_([this](const string& line) {
              RCLCPP_INFO(get_logger(), "%s", line.c_str());
          }) {
        brain_.init(*this);

        // Motion 的里程计：机器人相对启动原点的平面位置和朝向。
        odometrySub_ = create_subscription<booster_interface::msg::Odometer>(
            "/motion/odometer", 1,
            [this](const booster_interface::msg::Odometer::SharedPtr msg) {
                if(!isfinite(msg->x) || !isfinite(msg->y) || !isfinite(msg->theta)) return;
                brain::Odometry value;
                value.x = msg->x; value.y = msg->y; value.yaw = msg->theta;
                brain_.odometry.write(value, inputTime());
            });

        // 关节反馈：serial/parallel 两组电机的位置、速度、估计力矩及诊断状态。
        jointSub_ = create_subscription<booster_interface::msg::LowState>(
            "/low_state", 1,
            [this](const booster_interface::msg::LowState::SharedPtr msg) {
                const double now = inputTime();
                brain::JointData value;
                value.serial = convertJoints(msg->motor_state_serial);
                value.parallel = convertJoints(msg->motor_state_parallel);
                brain_.joint.write(value, now);
            });

        // 裁判信息：比赛阶段、双方队伍与比分，各球员的处罚状态。
        gameSub_ = create_subscription<game_controller_interface::msg::GameControlData>(
            "/robocup/game_controller", 1,
            [this](const game_controller_interface::msg::GameControlData::SharedPtr msg) {
                gc::Packet value;
                for(unsigned i=0;i<4;++i) value.header[i]=msg->header[i];
                value.version=msg->version; value.number=msg->packet_number; value.playersPerTeam=msg->players_per_team;
                value.competitionType=msg->competition_type; value.stopped=msg->stopped; value.phase=msg->game_phase;
                value.state=msg->state; value.setPlay=msg->set_play; value.firstHalf=msg->first_half; value.kickingTeam=msg->kicking_team;
                value.secsRemaining=msg->secs_remaining; value.secondaryTime=msg->secondary_time;
                for(unsigned i=0;i<2;++i) {
                    const auto& from=msg->teams[i]; gc::Team& to=value.teams[i];
                    to.number=from.team_number; to.fieldColour=from.field_player_colour; to.keeperColour=from.goalkeeper_colour;
                    to.goalkeeper=from.goalkeeper; to.score=from.score; to.penaltyShot=from.penalty_shot;
                    to.singleShots=from.single_shots; to.messageBudget=from.message_budget;
                    for(unsigned j=0;j<gc::maxPlayers;++j) {
                        to.players[j].penalty=from.players[j].penalty;
                        to.players[j].secsTillUnpenalised=from.players[j].secs_till_unpenalised;
                        to.players[j].cautions=from.players[j].cautions;
                    }
                }
                const double received=msg->received_at;
                if(!isfinite(received) || received<0 || received>inputTime()) value=gc::Packet();
                brain_.game.write(value,isfinite(received) ? received : 0);
            });

        // 物体观测：视觉识别出的球、机器人、场地标志点等物体及其位置、类别和置信度。
        visionSub_ = create_subscription<vision_interface::msg::Detections>(
            "/booster_vision/detection", 1,
            [this](const vision_interface::msg::Detections::SharedPtr msg) {
                brain::VisionState value;
                value.sourceTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                value.objects = static_cast<unsigned>(msg->detected_objects.size());
                for(const auto& object : msg->detected_objects) {
                    if(object.label == "Ball") ++value.balls;
                    if(object.label == "Opponent") ++value.robots;
                    brain::Detection detection;
                    detection.label = object.label; detection.color = object.color;
                    detection.confidence = object.confidence;
                    detection.box = {{static_cast<double>(object.xmin), static_cast<double>(object.ymin),
                                      static_cast<double>(object.xmax), static_cast<double>(object.ymax)}};
                    detection.projection = object.position_projection;
                    detection.pixel = object.target_uv;
                    value.detections.push_back(detection);
                }
                brain_.vision.write(value, inputTime());
            });

        // 场地线观测：视觉检测到的线段端点，包含机器人坐标和图像像素坐标，不带场地线身份。
        linesSub_ = create_subscription<vision_interface::msg::LineSegments>(
            "/booster_vision/line_segments", 1,
            [this](const vision_interface::msg::LineSegments::SharedPtr msg) {
                brain::FieldLines value;
                value.sourceTime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                value.coordinates = msg->coordinates;
                value.pixels = msg->coordinates_uv;
                brain_.fieldLines.write(value, inputTime());
            });

        // 头部位姿：头部相对机身的三维位置和四元数朝向，不是单独的头部关节角。
        headSub_ = create_subscription<geometry_msgs::msg::Pose>(
            "/head_pose", 1, [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                brain::HeadPose value;
                value.position = {{msg->position.x, msg->position.y, msg->position.z}};
                value.rotation = {{msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w}};
                brain_.headPose.write(value, inputTime());
            });

        // 倒地恢复状态：就绪、正在倒地、已倒地或正在起身，以及恢复可用性和当前规划器编号。
        recoverySub_ = create_subscription<booster_interface::msg::RawBytesMsg>(
            "fall_down_recovery_state", 1,
            [this](const booster_interface::msg::RawBytesMsg::SharedPtr msg) {
                if(msg->msg.size() < 3 || msg->msg[0] > 3) return;
                brain::RecoveryState value;
                value.state = msg->msg[0]; value.available = msg->msg[1] != 0; value.planner = msg->msg[2];
                brain_.recovery.write(value, inputTime());
            });
            
        // Brain 按固定节奏处理当前信息，消息到达不会直接触发算法。
        // 定时器和订阅回调在同一执行器中串行运行，这里不额外创建工作线程。
        tickTimer_ = create_wall_timer(chrono::milliseconds(10),
                                      [this]() { brain_.tick(inputTime()); });
    }

private:
    brain::Brain brain_;
    rclcpp::TimerBase::SharedPtr tickTimer_;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometrySub_;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr jointSub_;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameSub_;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr visionSub_;
    rclcpp::Subscription<vision_interface::msg::LineSegments>::SharedPtr linesSub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headSub_;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recoverySub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}
