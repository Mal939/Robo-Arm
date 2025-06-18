#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arm_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = create_service<arm_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::service_callback, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Service add_two_ints has been started");
    }

private:
    rclcpp::Service<arm_msgs::srv::AddTwoInts>::SharedPtr service_;

    void service_callback(const std::shared_ptr<arm_msgs::srv::AddTwoInts::Request> request, 
                          const std::shared_ptr<arm_msgs::srv::AddTwoInts::Response> response)
    {
        RCLCPP_INFO_STREAM(get_logger(), "New Request Received a: " << request->a << " b: " << request->b);
        response->sum = request->a + request->b;
        RCLCPP_INFO_STREAM(get_logger(), "Returning sum: " << response->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleServiceServer>());
    rclcpp::shutdown();
    return 0;
}