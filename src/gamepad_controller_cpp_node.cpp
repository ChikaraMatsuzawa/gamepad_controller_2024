#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

class Joystick : public rclcpp::Node{
  public:
    Joystick()
    : Node("gamepad_controller_cpp_node"),
      gear_position(1),
      shiftup_pressed(false),
      shiftdown_pressed(false)
    {
      subscription_ = create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&Joystick::callback, this, std::placeholders::_1));
      publisher_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 1);
      load_parameters();
    }

  private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr msg){
      auto cmd = process_message(*msg);
      publisher_->publish(cmd);
    }
    
    autoware_auto_control_msgs::msg::AckermannControlCommand process_message(const sensor_msgs::msg::Joy& msg){
      float input_steer_deg = msg.axes[steering_axis_index] * 450;
      float input_throttle = ((throttle_inverse ? -1.0 * msg.axes[throttle_axis_index] : msg.axes[throttle_axis_index]) + 1.0) / 2;
      float input_brake = ((brake_inverse ? -1.0 * msg.axes[brake_axis_index] : msg.axes[brake_axis_index]) + 1.0) / 2;

      // std::cout << "steer: " << input_steer_deg << std::endl;
      // std::cout << "throttle: " << input_throttle << std::endl;
      // std::cout << "brake: " << input_brake << std::endl;
    
      autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
      cmd.lateral.steering_tire_angle = input_steer_deg * 3.14159 / 360;
      if(input_brake > 0.01){
        cmd.longitudinal.acceleration = -10.0 * input_brake;
      }else if(input_throttle > 0.01){
        cmd.longitudinal.acceleration = 10.0 * input_throttle;
      }

      return cmd;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr publisher_;
    int gear_position;
    bool shiftup_pressed;
    bool shiftdown_pressed;
    int throttle_axis_index;
    int brake_axis_index;
    int steering_axis_index;
    int shiftup_button_index;
    int shiftdown_button_index;
    bool throttle_inverse;
    bool brake_inverse;
    void load_parameters(){
      declare_parameter("throttle_axis_index", 2);
      declare_parameter("brake_axis_index", 1);
      declare_parameter("steering_axis_index", 0);
      declare_parameter("shiftup_button_index", 1);
      declare_parameter("shiftdown_button_index", 0);
      declare_parameter("throttle_inverse", false);
      declare_parameter("brake_inverse", false);
      throttle_axis_index = get_parameter("throttle_axis_index").as_int();
      brake_axis_index = get_parameter("brake_axis_index").as_int();
      steering_axis_index = get_parameter("steering_axis_index").as_int();
      shiftup_button_index = get_parameter("shiftup_button_index").as_int();
      shiftdown_button_index = get_parameter("shiftdown_button_index").as_int();
      throttle_inverse = get_parameter("throttle_inverse").as_bool();
      brake_inverse = get_parameter("brake_inverse").as_bool();
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Joystick>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
