#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_turtle_commands_dop/action/message_turtle_commands.hpp"

class ActionTurtleClient : public rclcpp::Node {
public:
    using MessageTurtleCommands = action_turtle_commands_dop::action::MessageTurtleCommands;
    using GoalHandleMessageTurtleCommands = rclcpp_action::ClientGoalHandle<MessageTurtleCommands>;

    ActionTurtleClient() : Node("action_turtle_client"), current_step(0) {
        this->client_ = rclcpp_action::create_client<MessageTurtleCommands>(this, "message_turtle_commands");
        send_goal(); // Начинаем отправку первой команды
    }

private:
    rclcpp_action::Client<MessageTurtleCommands>::SharedPtr client_;
    int current_step; // Переменная для отслеживания текущего шага

    void send_goal() {
        auto goal_msg = MessageTurtleCommands::Goal();

        // Определение команд в зависимости от текущего шага
        if (current_step == 0) {
            goal_msg.command = "forward";
            goal_msg.s = 2; // Увеличим количество шагов для движения вперед
            goal_msg.angle = 0;
        } else if (current_step == 1) {
            goal_msg.command = "turn_right";
            goal_msg.s = 1; // Поворачиваем на 90 градусов
            goal_msg.angle = 90;
        } else if (current_step == 2) {
            goal_msg.command = "forward";
            goal_msg.s = 1; // Увеличим количество шагов для движения вперед
            goal_msg.angle = 0;
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal: %s", goal_msg.command.c_str());
        auto send_goal_options = rclcpp_action::Client<MessageTurtleCommands>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&ActionTurtleClient::result_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&ActionTurtleClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedback_callback(GoalHandleMessageTurtleCommands::SharedPtr goal_handle, const std::shared_ptr<const typename MessageTurtleCommands::Feedback> feedback) {
        if (feedback) {
            RCLCPP_INFO(this->get_logger(), "Current odometry: %d meters", feedback->odom);
        }
    }

    void result_callback(const GoalHandleMessageTurtleCommands::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            current_step++; // Переходим к следующему шагу
            if (current_step < 3) {
                send_goal(); // Отправляем следующую команду
            } else {
                RCLCPP_INFO(this->get_logger(), "All goals completed.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal failed");
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ActionTurtleClient>();
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
