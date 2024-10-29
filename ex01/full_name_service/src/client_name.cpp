#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "full_name_message/srv/full_name_sum_service.hpp"

using namespace std::chrono_literals;

class NameClient : public rclcpp::Node {
public:
    NameClient(const std::string &last_name, const std::string &name, const std::string &first_name)
        : Node("name_client") {
        client_ = this->create_client<full_name_message::srv::FullNameSumService>("SummFullName");

        auto request = std::make_shared<full_name_message::srv::FullNameSumService::Request>();
        request->last_name = last_name;
        request->name = name;
        request->first_name = first_name;

        // Ждем, пока сервис станет доступен
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        // Отправляем запрос
        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Полное имя: %s", result.get()->full_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Не удалось выполнить запрос к сервису");
        }
    }

private:
    rclcpp::Client<full_name_message::srv::FullNameSumService>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: client_name <last_name> <name> <first_name>");
        return 1;
    }

    NameClient client(argv[1], argv[2], argv[3]);
    rclcpp::shutdown();
    return 0;
}