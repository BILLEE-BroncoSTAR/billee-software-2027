#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

class JoyTankDrive : public rclcpp::Node{

    public:
        JoyTankDrive() : Node("joy_tank_drive"){
            leftAxisId = this->declare_parameter("left_axis", 0);
            rightAxisId = this->declare_parameter("right_axis",2);
            trackWidth = this->declare_parameter("track_width",.67);
            safetyButtonId = this->declare_parameter("safety_button", 5);
            // Deadman on an analog trigger instead of a button: safety_axis >= 0 uses that
            // axis, held while it reads below safety_axis_threshold (Xbox LT/RT: +1 released,
            // -1 fully pressed, and 0 from joy_node until first touched).
            safetyAxisId = this->declare_parameter("safety_axis", -1);
            safetyAxisThreshold = this->declare_parameter("safety_axis_threshold", -0.5);

            maxVel = this->declare_parameter("max_vel", 0.0);



            // set depth of 1 so newly arrived packets immediately override old ones
            joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&JoyTankDrive::topic_callback, this, std::placeholders::_1));
            twistPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        }

    private:
        void topic_callback(const sensor_msgs::msg::Joy& msg){
            auto twistMsg = geometry_msgs::msg::Twist();
            const auto nAxes = static_cast<int>(msg.axes.size());
            const auto nButtons = static_cast<int>(msg.buttons.size());
            if (leftAxisId < 0 || leftAxisId >= nAxes || rightAxisId < 0 || rightAxisId >= nAxes) {
                twistPublisher_->publish(twistMsg);  // malformed /joy: stop
                return;
            }
            float leftAxis = msg.axes[leftAxisId];
            float rightAxis = msg.axes[rightAxisId];

            bool safetyButton = false;
            if (safetyAxisId >= 0) {
                safetyButton = safetyAxisId < nAxes && msg.axes[safetyAxisId] < safetyAxisThreshold;
            } else if (safetyButtonId >= 0 && safetyButtonId < nButtons) {
                safetyButton = msg.buttons[safetyButtonId] != 0;
            }

            RCLCPP_DEBUG(this->get_logger(), "Got Axes Vals: %f %f", leftAxis, rightAxis);
            RCLCPP_DEBUG(this->get_logger(), "Safety Button Reading: %d", safetyButton);

            if(safetyButton){
                twistMsg.linear.x = ((leftAxis + rightAxis) / 2.0) * maxVel;
                twistMsg.linear.y = 0;
                twistMsg.linear.z = 0;
                twistMsg.angular.x = 0;
                twistMsg.angular.y =  0;
                twistMsg.angular.z = ((rightAxis - leftAxis) / trackWidth) * maxVel;
            }

            twistPublisher_->publish(twistMsg);

        }

        int leftAxisId;
        int rightAxisId;
        int safetyButtonId;
        int safetyAxisId;
        double safetyAxisThreshold;

        float maxVel;
        float trackWidth;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher_;

};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTankDrive>());
    rclcpp::shutdown();

    return 0;
}