#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>

class KeyboardController : public rclcpp::Node {
public:
    KeyboardController() : Node("keyboard_controller") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
        
        // Initialize keyboard settings
        tcgetattr(STDIN_FILENO, &initial_settings_);
        termios new_settings = initial_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VMIN] = 0;  // Non-blocking read
        new_settings.c_cc[VTIME] = 0; 
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

        RCLCPP_INFO(this->get_logger(), "Use W/A/S/D for movement, Q to quit");
        loop();
    }

    ~KeyboardController() {
        tcsetattr(STDIN_FILENO, TCSANOW, &initial_settings_);
    }

private:
    void loop() {
        double linear_vel = 0.2;
        double angular_vel = 0.2;
        geometry_msgs::msg::Twist twist;

        while (rclcpp::ok()) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                switch(c) {
                    case 'w':
                        twist.linear.x = std::min(twist.linear.x + linear_vel, 5.0);
                        break;
                    case 's':
                        twist.linear.x = std::max(twist.linear.x - linear_vel, -5.0);
                        break;
                    case 'a':
                        twist.angular.z = std::min(twist.angular.z + angular_vel, 1.0);
                        break;
                    case 'd':
                        twist.angular.z = std::max(twist.angular.z - angular_vel, -1.0);
                        break;
                    case 'q':
                        rclcpp::shutdown();
                        return;
                }
                publisher_->publish(twist);
            }
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    struct termios initial_settings_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>();
    rclcpp::spin(node);
    return 0;
}