#include "rclcpp/rclcpp.hpp"
#include "full_name_message/srv/full_name_sum_service.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NameService : public rclcpp::Node {
public:
    NameService() : Node("name_service") {
        service_ = this->create_service<full_name_message::srv::FullNameSumService>(
            "SummFullName", std::bind(&NameService::handle_request, this, _1, _2));
    }

private:
    void handle_request(const std::shared_ptr<full_name_message::srv::FullNameSumService::Request> request,
                        std::shared_ptr<full_name_message::srv::FullNameSumService::Response> response) {
        // Склеиваем строки и формируем полное имя
        response->full_name = request->last_name + " " + request->name + " " + request->first_name;
        RCLCPP_INFO(this->get_logger(), "Составлено полное имя: %s", response->full_name.c_str());
    }

    rclcpp::Service<full_name_message::srv::FullNameSumService>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NameService>());
    rclcpp::shutdown();
    return 0;
}