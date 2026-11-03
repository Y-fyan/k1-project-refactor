#pragma once
#include <chrono>
#include <string>
#include <vector>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <game_controller_interface/protocol.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <game_controller_interface/msg/game_control_return_data.hpp>

class GameControllerNode : public rclcpp::Node {
public:
    explicit GameControllerNode(std::string name);
    ~GameControllerNode();
    void init();
private:
    static double now();
    void poll();
    void publish(const gc::Packet& packet);
    int socket_ = -1, port_ = 3838, expectedTeam_ = 0;
    bool whitelist_ = false;
    std::vector<std::string> allowedIPs_;
    sockaddr_in source_{};
    gc::Sequence sequence_;
    gc::Packet packet_;
    gc::ReturnData feedback_;
    double packetAt_ = 0, feedbackAt_ = 0, sentAt_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<game_controller_interface::msg::GameControlData>::SharedPtr publisher_;
    rclcpp::Subscription<game_controller_interface::msg::GameControlReturnData>::SharedPtr feedbackSub_;
};
