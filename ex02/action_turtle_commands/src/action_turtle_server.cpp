#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "action_turtle_commands_dop/action/message_turtle_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class ActionTurtleServer : public rclcpp::Node {
public:
    using MessageTurtleCommands = action_turtle_commands_dop::action::MessageTurtleCommands;
    using GoalHandleMessageTurtleCommands = rclcpp_action::ServerGoalHandle<MessageTurtleCommands>;

    static std::shared_ptr<ActionTurtleServer> create() {
        auto server = std::shared_ptr<ActionTurtleServer>(new ActionTurtleServer());
        server->init_action_server(); // Инициализируем сервер действий здесь
        return server;
    }

private:
    ActionTurtleServer() : Node("action_turtle_server") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&ActionTurtleServer::pose_callback, this, std::placeholders::_1));
    }

    void init_action_server() {
        auto server_ptr = shared_from_this(); // Получаем shared_ptr на текущий объект
        action_server_ = rclcpp_action::create_server<MessageTurtleCommands>(
            server_ptr,
            "message_turtle_commands",
            std::bind(&ActionTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionTurtleServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionTurtleServer::handle_accepted, this, std::placeholders::_1));
    }

    rclcpp_action::Server<MessageTurtleCommands>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    turtlesim::msg::Pose current_pose_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & /*uuid*/, 
        std::shared_ptr<const MessageTurtleCommands::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request: command=%s, s=%d, angle=%d",
                    goal->command.c_str(), goal->s, goal->angle);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMessageTurtleCommands> /*goal_handle*/) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle) {
        auto server_ptr = std::dynamic_pointer_cast<ActionTurtleServer>(shared_from_this());
        std::thread{[server_ptr, goal_handle]() {
            if (server_ptr) {
                server_ptr->execute(goal_handle);
            }
        }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMessageTurtleCommands> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MessageTurtleCommands::Feedback>();
        feedback->odom = 0;

        geometry_msgs::msg::Twist msg;
        rclcpp::Rate loop_rate(10); // 10 Гц

        double last_x = current_pose_.x;
        double last_y = current_pose_.y;
        double distance_traveled = 0.0;

        if (goal->command == "forward") {
            msg.linear.x = 0.5; // Установите скорость вперед
            double target_distance = goal->s; // Целевое расстояние

            while (distance_traveled < target_distance) {
                if (goal_handle->is_canceling()) {
                    auto result = std::make_shared<MessageTurtleCommands::Result>();
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                cmd_vel_pub_->publish(msg);
                loop_rate.sleep();

                // Вычисляем пройденное расстояние
                distance_traveled += std::sqrt(std::pow(current_pose_.x - last_x, 2) + 
                                                std::pow(current_pose_.y - last_y, 2));
                last_x = current_pose_.x;
                last_y = current_pose_.y;

                feedback->odom = distance_traveled; // Обновляем информацию об одометрии
                goal_handle->publish_feedback(feedback);
            }
        } else if (goal->command == "turn_left") {
            msg.angular.z = 1.0; // Установите угловую скорость для поворота влево
            double target_angle = current_pose_.theta + (goal->angle * M_PI / 180.0); // Целевой угол в радианах

            while (current_pose_.theta < target_angle) {
                if (goal_handle->is_canceling()) {
                    auto result = std::make_shared<MessageTurtleCommands::Result>();
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                cmd_vel_pub_->publish(msg);
                feedback->odom = current_pose_.theta; // Обновляем информацию об угле
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }
        } else if (goal->command == "turn_right") {
            msg.angular.z = -1.0; // Установите угловую скорость для поворота вправо
            double target_angle = current_pose_.theta - (goal->angle * M_PI / 180.0); // Целевой угол в радианах

            while (current_pose_.theta > target_angle) {
                if (goal_handle->is_canceling()) {
                    auto result = std::make_shared<MessageTurtleCommands::Result>();
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }

                cmd_vel_pub_->publish(msg);
                feedback->odom = current_pose_.theta; // Обновляем информацию об угле
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }
        }

        // Завершаем движение
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        cmd_vel_pub_->publish(msg);

        auto result = std::make_shared<MessageTurtleCommands::Result>();
        result->result = true;
        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto server = ActionTurtleServer::create(); // Используем статический метод для создания экземпляра
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}
