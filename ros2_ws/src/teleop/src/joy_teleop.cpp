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

            maxVel = this->declare_parameter("max_vel", 0.0);



            // set depth of 1 so newly arrived packets immediately override old ones
            joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&JoyTankDrive::topic_callback, this, std::placeholders::_1));
            twistPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        }

    private:
        void topic_callback(const sensor_msgs::msg::Joy& msg){
            float leftAxis = msg.axes[leftAxisId];
            float rightAxis = msg.axes[rightAxisId];

            int safetyButton = msg.buttons[safetyButtonId];

            auto twistMsg = geometry_msgs::msg::Twist();

            RCLCPP_DEBUG(this->get_logger(), "Got Axes Vals: %f %f", leftAxis, rightAxis);
            RCLCPP_DEBUG(this->get_logger(), "Safety Button Reading: %d", safetyButton);

            if(safetyButton){
                twistMsg.linear.x = ((leftAxis + rightAxis) / 2.0) * maxVel;
                twistMsg.linear.y = 0;
                twistMsg.linear.z = 0;
                twistMsg.angular.x = 0;
                twistMsg.angular.y =  0;
                twistMsg.angular.z = ((rightAxis - leftAxis) / trackWidth) * maxVel;
            } else if(!wasHeld){
                // Deadman released: publish nothing, so a command mux (twist_mux) times this
                // source out and lets lower-priority sources (Nav2) drive. diff_drive_controller
                // stops on its own cmd_vel_timeout when nothing else publishes.
                return;
            }
            // Held: the stick command. Just released: one zero, so the rover stops at once.
            wasHeld = safetyButton != 0;

            twistPublisher_->publish(twistMsg);

        }

        bool wasHeld = false;
        int leftAxisId;
        int rightAxisId;
        int safetyButtonId;

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