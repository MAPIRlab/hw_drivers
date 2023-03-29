#include <functional>
#include <memory>
#include <thread>
#include <time.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "braccio/srv/status.hpp"
#include "braccio/action/braccio_cmd.hpp"

class Braccio: public rclcpp::Node{
public:
    Braccio();
    ~Braccio();

    // some definitions to make live easier
    using BraccioAction = braccio::action::BraccioCMD;
    using GoalHandleBraccio = rclcpp_action::ServerGoalHandle<BraccioAction>;

private:
    // Srv server
    rclcpp::Service<braccio::srv::Status>::SharedPtr srv_server_;

    // Action server
    rclcpp_action::Server<braccio::action::BraccioCMD>::SharedPtr action_server_;

    // Srv Callbacks
    void process_service_request(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<braccio::srv::Status::Request> request,
        std::shared_ptr<braccio::srv::Status::Response> response);

    // Action Callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const BraccioAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBraccio> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleBraccio> goal_handle);

    void execute(const std::shared_ptr<GoalHandleBraccio> goal_handle);

    // Serial & functions
    bool config_serial(std::string portName);
    geometry_msgs::msg::Point braccio_mcd(std::vector<float> joints);
    std::vector<float> braccio_mci(geometry_msgs::msg::Point p);

    // Internal variables
    geometry_msgs::msg::Point current_position;
    std::vector<float> current_joints;
    int serial_port;
    std::string serialPortFilename;

    // Arm Link lengths (m)
    float L1, L2, L3, L4, L5;
};

