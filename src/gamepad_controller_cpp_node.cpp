#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>

class GamepadController : public rclcpp::Node{
  public:
    GamepadController()
    : Node("gamepad_controller_cpp_node"),
      gear_position(3),
      shiftup_pressed(false),
      shiftdown_pressed(false)
    {
      subscription_ = create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&GamepadController::callback, this, std::placeholders::_1));
      publisher1_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/control/command/control_cmd", 1);
      publisher2_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1);
      load_parameters();
    }

  private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr msg){
      auto control_cmd = msg_to_control_cmd(*msg);
      publisher1_->publish(control_cmd);
      auto gear_cmd = msg_to_gear_command(*msg);
      publisher2_->publish(gear_cmd);
    }
    
    autoware_auto_control_msgs::msg::AckermannControlCommand msg_to_control_cmd(const sensor_msgs::msg::Joy& msg){
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

    autoware_auto_vehicle_msgs::msg::GearCommand msg_to_gear_command(const sensor_msgs::msg::Joy& msg){
      if(msg.buttons[shiftup_button_index] == 1){
        if(gear_position < 3 && !shiftup_pressed){
          gear_position++;
        }
        shiftup_pressed = true;
      }else{
        shiftup_pressed = false;
      }
      if(msg.buttons[shiftdown_button_index] == 1){
        if(gear_position > 0 && !shiftdown_pressed){
          gear_position--;
        }
        shiftdown_pressed = true;
      }else{
        shiftdown_pressed = false;
      }

      std::cout << "gear_position: " << gear_position << std::endl;

      autoware_auto_vehicle_msgs::msg::GearCommand cmd;
      if(gear_position == 0){
        cmd.command = 22;
      }else if(gear_position == 1){
        cmd.command = 20;
      }else if(gear_position == 2){
        cmd.command = 1;
      }else{
        cmd.command = 2;
      }

      return cmd;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr publisher1_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr publisher2_;
    int gear_position; // 0:PARK, 1:REVERSE, 2:NEUTRAL, 3:DRIVE
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
    auto node = std::make_shared<GamepadController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
