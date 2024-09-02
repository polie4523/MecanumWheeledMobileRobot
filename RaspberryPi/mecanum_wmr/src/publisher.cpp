#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class Publisher : public rclcpp::Node {
    public:
        Publisher() : Node("publisher") {
            publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/target_position", 10);
            
        }
        void SendTargetPoint(const float x, const float y, const float theta) {
            
            auto message = geometry_msgs::msg::Point();
            message.x = (double) x;
            message.y = (double) y;
            message.z = (double) theta;
            publisher_->publish(message);
            
            
        }
    private:
        
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        
        
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>();
    //rclcpp::spin();
    node->SendTargetPoint(1,0,0);
    rclcpp::shutdown();
    return 0;
}
