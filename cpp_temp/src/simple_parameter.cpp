// Include the main ROS 2 C++ library — it gives you access to ROS nodes, logging, parameters, etc.
#include <rclcpp/rclcpp.hpp>

// This lets you return a success or failure result when parameters change.
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// These are standard C++ libraries:
#include <string>   // string lets you use text (e.g., names, words)
#include <vector>   // vector is like a list or array
#include <memory>   // memory helps manage smart pointers (automatic memory management)

// This allows you to use _1 as a placeholder for function arguments when binding a callback.
using std::placeholders::_1;


// This defines a new class called SimpleParameter.
// It inherits from rclcpp::Node, which means it's a ROS 2 node.
// A node in ROS is a single program that performs some tasks.
class SimpleParameter : public rclcpp::Node
{
public:
    // This is a constructor — it runs automatically when the object is created.
    SimpleParameter() : Node("simple_parameter")
    {
        // Creates a parameter named simple_int_param with a default value of 28.
        declare_parameter<int>("simple_int_param", 28);

        // Creates another parameter named simple_string_param with the default value "Malek".
        declare_parameter<std::string>("simple_string_param", "Malek");

        // This tells ROS what function to call when parameters are changed while the program is running.
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

private:
    // This is a pointer that stores the handle for the callback,
    // so ROS knows where to call when parameters change.
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // This function is called automatically when a user tries to change a parameter.
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        // This creates a result message to return to ROS.
        rcl_interfaces::msg::SetParametersResult result;

        // This loops through all changed parameters. It checks the name and type.
        for(const auto& param : parameters)
        {
            // If it's the integer parameter, it prints out its new value.
            if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! New value is: " << param.as_int());
                result.successful = true;
            }

            // Same idea for the string parameter.
            if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is: " << param.as_string());
                result.successful = true;
            }
        }

        // Finally, it returns a result to tell ROS that the parameter change was successful.
        return result;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
