#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <can_msgs/msg/can_msg.hpp>

using namespace std::chrono_literals;

/**
 * @brief remote emergency stop
 * 
 */
class res : public rclcpp::Node
{
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub;
	rclcpp::Publisher<can_msgs::msg::CanMsg>::SharedPtr m_can_pub;

public:
	res() : Node("res")
	{
		m_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&res::joy_callback, this, std::placeholders::_1));
        m_can_pub = this->create_publisher<can_msgs::msg::CanMsg>("slcan", 10);
	}

private:
	void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		static bool pre_emergency = false;
		const auto rise_emergency =  msg->buttons[10] ^ pre_emergency & msg->buttons[10];
		pre_emergency = msg->buttons[10];

		static bool is_emergency = false;
		if (rise_emergency ^ is_emergency)
		{
			can_msgs::msg::CanMsg can_msg;
			can_msg.is_extended = true;
			
			can_msg.id = 0x00F0000E;

			m_can_pub->publish(can_msg);
			is_emergency = true;
		}
		else if(rise_emergency)
		{
			can_msgs::msg::CanMsg can_msg;
            can_msg.is_extended = true;
            
            can_msg.id = 0x00F0000F;

            m_can_pub->publish(can_msg);
			is_emergency = false;
		}

		RCLCPP_INFO(this->get_logger(), "emergency: %s", is_emergency ? "true" : "false");
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<res>());
    rclcpp::shutdown();
    return 0;
}