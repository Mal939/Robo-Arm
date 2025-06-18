#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arm_msgs/srv/add_two_ints.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient : public rclcpp::Node
{
public:
    SimpleServiceClient(int a, int b) : Node("simple_service_client")
    {
        client_ = create_client<arm_msgs::srv::AddTwoInts>("add_two_ints");
        auto request = std::make_shared<arm_msgs::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        while (rclcpp::ok() && !client_->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
            return;
        }

        auto result = client_->async_send_request(
            request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }

private:
    rclcpp::Client<arm_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<arm_msgs::srv::AddTwoInts>::SharedFuture future)
    {
        auto result = future.get();
        RCLCPP_INFO_STREAM(get_logger(), "Service Response: " << result->sum);
    }
};

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " a b" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}