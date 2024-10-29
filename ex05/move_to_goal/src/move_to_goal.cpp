#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MoveToGoal : public rclcpp::Node {
public:
    MoveToGoal() : Node("move_to_goal") {
        // Получаем параметры x, y, theta из командной строки
        this->declare_parameter("goal_x", 0.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("goal_theta", 0.0);
        
        this->get_parameter("goal_x", goal_x_);
        this->get_parameter("goal_y", goal_y_);
        this->get_parameter("goal_theta", goal_theta_);

        // Подписка на позицию черепахи
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MoveToGoal::pose_callback, this, std::placeholders::_1));

        // Публикация команд скорости
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(100ms, std::bind(&MoveToGoal::timer_callback, this));
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
    }

    void timer_callback() {
        double distance = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);
        if (distance > 0.01) { // Если черепаха не достигла цели
            double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
            double angle_diff = angle_to_goal - current_theta_;
            if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            geometry_msgs::msg::Twist msg;
            msg.linear.x = 1.0 * distance; // Устанавливаем линейную скорость
            msg.angular.z = 4.0 * angle_diff; // Устанавливаем угловую скорость
            publisher_->publish(msg);
        } else {
            // Остановка черепахи при достижении цели
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
            rclcpp::shutdown(); // Завершение ноды
        }
    }

    double goal_x_;
    double goal_y_;
    double goal_theta_;
    double current_x_;
    double current_y_;
    double current_theta_;
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveToGoal>());
    rclcpp::shutdown();
    return 0;
}
