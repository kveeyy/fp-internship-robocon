#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <unordered_map>
#include <string>

class TeleopKeyboardNode : public rclcpp::Node {
public:
    TeleopKeyboardNode() : Node("teleop_keyboard_node") {
        // Publisher untuk mengirimkan perintah ke robot
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Menampilkan kontrol
        show_instructions();

        // Loop untuk membaca input keyboard dan mengontrol robot
        controlLoop();
    }

private:
    // Fungsi untuk menampilkan petunjuk kontrol
    void show_instructions() {
        std::cout << "Control Your Robot with the Keyboard!" << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std::cout << "Move:        i" << std::endl;
        std::cout << "Reverse:     ," << std::endl;
        std::cout << "Left:        j" << std::endl;
        std::cout << "Right:       l" << std::endl;
        std::cout << "Stop:        k" << std::endl;
        std::cout << "Exit:        q" << std::endl;
        std::cout << "------------------------------------" << std::endl;
    }

    // Fungsi utama untuk membaca input keyboard dan mengontrol robot
    void controlLoop() {
        geometry_msgs::msg::Twist msg;
        char key;

        // Kecepatan linear dan angular
        const double linear_speed = 0.5;
        const double angular_speed = 1.0;

        // Peta key mapping ke kecepatan
        std::unordered_map<char, std::pair<double, double>> key_mapping = {
            {'i', {linear_speed, 0.0}},   // Maju
            {',', {-linear_speed, 0.0}}, // Mundur
            {'j', {0.0, angular_speed}}, // Belok kiri
            {'l', {0.0, -angular_speed}},// Belok kanan
            {'k', {0.0, 0.0}}            // Berhenti
        };

        while (rclcpp::ok()) {
            std::cout << "Enter command: ";
            std::cin >> key;

            // Cek apakah key valid
            if (key == 'q') {
                std::cout << "Exiting teleop control." << std::endl;
                break;
            } else if (key_mapping.find(key) != key_mapping.end()) {
                // Ambil kecepatan linear dan angular dari key mapping
                msg.linear.x = key_mapping[key].first;
                msg.angular.z = key_mapping[key].second;
                publisher_->publish(msg);
            } else {
                std::cout << "Invalid command! Use 'i', ',', 'j', 'l', 'k', or 'q'." << std::endl;
            }

            // Untuk memberikan waktu jeda pada input keyboard
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
