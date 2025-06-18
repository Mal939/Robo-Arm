#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arm_msgs/srv/euler_to_quaternion.hpp>
#include <arm_msgs/srv/quaternion_to_euler.hpp>

#include <tf2/utils.h>

using namespace std::placeholders;

class AnglesConverter : public rclcpp::Node
{
public:
    AnglesConverter() : Node("angles_convertion_serive")
    {
        euler_to_quaternion_ = create_service<arm_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AnglesConverter::euler_to_quaternion_callback, this, _1, _2));
        quaternion_to_euler_ = create_service<arm_msgs::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AnglesConverter::quaternion_to_euler_callback, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Angle Conversion Services are Ready");
    }

private:
    rclcpp::Service<arm_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
    rclcpp::Service<arm_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

    void euler_to_quaternion_callback(const std::shared_ptr<arm_msgs::srv::EulerToQuaternion::Request> request,        
                                      const std::shared_ptr<arm_msgs::srv::EulerToQuaternion::Response> response)
    {
        RCLCPP_INFO_STREAM(get_logger(), "New Request Received roll: " << request->roll << " pitch: " << request->pitch << " yaw: " << request->yaw << " into a quaternion");
        tf2::Quaternion q;
        q.setRPY(request->roll, request->pitch, request->yaw);
        response->x = q.getX();
        response->y = q.getY();
        response->z = q.getZ();
        response->w = q.getW();
        RCLCPP_INFO_STREAM(get_logger(), "Returning quaternion x: " << response->x << " y: " << response->y << " z: " << response->z << " w: " << response->w);

    }
    void quaternion_to_euler_callback(const std::shared_ptr<arm_msgs::srv::QuaternionToEuler::Request> request,        
                                      const std::shared_ptr<arm_msgs::srv::QuaternionToEuler::Response> response)
    {
        RCLCPP_INFO_STREAM(get_logger(), "New Quaternion Request Received x: " << request->x << " y: " << request->y << " z: " << request->z << " w: " << request->w << " into an euler");
        tf2::Quaternion q(request->x, request->y, request->z, request->w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); //get roll, pitch, yaw
        response->roll = roll;
        response->pitch = pitch;
        response->yaw = yaw;
        RCLCPP_INFO_STREAM(get_logger(), "Returning Euler Angles\n roll: " << response->roll << " pitch: " << response->pitch << " yaw: " << response->yaw);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnglesConverter>());
    rclcpp::shutdown();
    return 0;
}