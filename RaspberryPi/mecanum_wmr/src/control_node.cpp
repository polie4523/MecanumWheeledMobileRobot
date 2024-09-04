#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "mecanum_wmr/MecanumWMR.h"


using std::placeholders::_1;
using namespace std::chrono_literals;


class ControlNode : public rclcpp::Node {
    public:
        ControlNode(MecanumWMR wmr): Node("control_node"), last_callback_time_(std::chrono::high_resolution_clock::now()) {
            
            topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = topic_cb_group_;

            subscription_ = this->create_subscription<geometry_msgs::msg::Point>("/target_position", 10, std::bind(&ControlNode::topic_callback, this, _1), sub_options);
            publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/position", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&ControlNode::timer_callback, this), timer_cb_group_);

            wmr_ = wmr;

            this->declare_parameter("init_cond", std::vector<double>{0.0, 0.0, 0.0});
            this->declare_parameter("pid_gain_translation", std::vector<double>{1, 0.25, 0.25});
            this->declare_parameter("pid_gain_rotation", std::vector<double>{1, 0.25, 0.25});

            rclcpp::Parameter init_cond_param = this->get_parameter("init_cond");
            rclcpp::Parameter pid_gain_translation_param = this->get_parameter("pid_gain_translation");
            rclcpp::Parameter pid_gain_rotation_param = this->get_parameter("pid_gain_rotation");
            auto init_cond = init_cond_param.as_double_array();
            auto pid_gain_translation = pid_gain_translation_param.as_double_array();
            auto pid_gain_rotation = pid_gain_rotation_param.as_double_array();

            wmr_.setInitCond(init_cond[0], init_cond[1], init_cond[2]);
            wmr_.setPIDgain_translation(pid_gain_translation[0], pid_gain_translation[1], pid_gain_translation[2]);
            wmr_.setPIDgain_rotation(pid_gain_rotation[0], pid_gain_rotation[1], pid_gain_rotation[2]);
            
            RCLCPP_INFO(this->get_logger(), "Waiting for topic...");
        }

    private:
        void topic_callback(const geometry_msgs::msg::Point & msg) {
            /*auto newtime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> time_diff = newtime - last_callback_time_;
            last_callback_time_ = newtime;
            RCLCPP_INFO(this->get_logger(), "Time since last callback: %.3f ms", time_diff.count());*/

            //auto start = std::chrono::high_resolution_clock::now();

            
            target_point[0] = (float) msg.x;
            target_point[1] = (float) msg.y;
            target_point[2] = (float) msg.z;
            RCLCPP_INFO(this->get_logger(), "Get target_point [x,y,theta]: [%.3f, %.3f, %.3f]",target_point[0],target_point[1],target_point[2]);

            timer_->reset();
            wmr_.Point2PointMove(target_point);
            timer_->cancel();

            auto p_est = wmr_.getPosition();
            RCLCPP_INFO(this->get_logger(), "Current position [x,y,theta]: [%.3f, %.3f, %.3f]", p_est[0], p_est[1], p_est[2]);
            /*auto message = geometry_msgs::msg::Point();
            message.x = (double) p_est[0];
            message.y = (double) p_est[1];
            message.z = (double) p_est[2];
            publisher_->publish(message);*/

            /*auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
            RCLCPP_INFO(this->get_logger(), "Callback execution time: %.3f ms", elapsed.count());*/

        }
        void timer_callback() {
            auto p_est = wmr_.getPosition();
            auto message = geometry_msgs::msg::Point();
            message.x = (double) p_est[0];
            message.y = (double) p_est[1];
            message.z = (double) p_est[2];
            publisher_->publish(message);
        }
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
        rclcpp::TimerBase::SharedPtr timer_;
        MecanumWMR wmr_;
        std::array<float, 3> target_point {0.0};
        std::chrono::high_resolution_clock::time_point last_callback_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    MecanumWMR wmr;
    if (wmr.init()==false) return 1;
    auto node = std::make_shared<ControlNode>(wmr);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    //rclcpp::spin(node);
    wmr.shutdown();
    rclcpp::shutdown();
    return 0;
}
