#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "braccio/action/braccio_cmd.hpp"


class BraccioActionClient : public rclcpp::Node
{
public:
  using BraccioAction = braccio::action::BraccioCMD;
  using GoalHandleBraccio = rclcpp_action::ClientGoalHandle<BraccioAction>;
  std::vector<float> joints_a = {0, 90, 90, 90, 90, 20};
  std::vector<float> joints_b = {90, 90, 90, 90, 90, 60};
  bool b = true;

  BraccioActionClient() : Node("braccio_action_client")
  {
    this->client_ptr_ = rclcpp_action::create_client<BraccioAction>(
      this,
      "braccio_action");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10000),
      std::bind(&BraccioActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;
    //this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = BraccioAction::Goal();

    // Set joints goal
    /*
    if (b)
      goal_msg.goal_joints = joints_a;
    else
      goal_msg.goal_joints = joints_b;
    */

    // Set Pose goal
    geometry_msgs::msg::Point p;
    if (b){
      p.x = 0.3;
      p.y = 0.2;
      p.z = -0.2;
    }else{
      p.x = -0.3;
      p.y = -0.2;
      p.z = -0.2;
    }
    goal_msg.goal_position = p;

    // change goal
    b = !b;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<BraccioAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&BraccioActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&BraccioActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&BraccioActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<BraccioAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleBraccio::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleBraccio::SharedPtr,
    const std::shared_ptr<const BraccioAction::Feedback> feedback)
  {
  }

  void result_callback(const GoalHandleBraccio::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    ss << result.result->goal_achieved;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    //rclcpp::shutdown();
  }
};



int main (int argc, char* argv[]){
    //inicializamos node
    rclcpp::init(argc,argv);
    auto p = std::make_shared<BraccioActionClient>();
    rclcpp::spin(p);

    //Al terminar cerramos todo
    rclcpp::shutdown();
    return 0;
}

