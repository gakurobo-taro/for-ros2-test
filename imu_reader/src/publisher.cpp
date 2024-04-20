#include <utility>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class imu_sender : public rclcpp::Node
{
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_imu_sub;
	rclcpp::TimerBase::SharedPtr m_timer;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;

	std::queue<std::string> m_buffer;

public:
	imu_sender() : Node("imu_sender")
	{
		m_imu_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
			"uart_rx_msg", 10, std::bind(&imu_sender::imu_callback, this, std::placeholders::_1));

		m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

		m_timer = this->create_wall_timer(1ms, std::bind(&imu_sender::timer_callback, this));
	}

private:
	auto imu_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) -> void
	{
		constinit static auto header = 'I';
		
		for(const auto& c : msg->data)
		{
			if(c == header)
				m_buffer.push(&header);
			else
				m_buffer.back() += c;
		}
	}

	auto timer_callback() -> void
	{
		constinit static auto header = 'I';

		if(m_buffer.empty())
		    return;

		if(m_buffer.front().substr(0, 2) == "I0")
			return;
		
	    // RCLCPP_INFO(this->get_logger(), "%s", m_buffer.front().substr());

		if(m_buffer.front().front() == header
			&& m_buffer.front().back() == '\n')
		{
			std::string tmp_string = std::move(m_buffer.front());
			m_buffer.pop();

			auto imu_msg = sensor_msgs::msg::Imu();

			imu_msg.header.stamp = this->now();

			if(tmp_string.substr(0, 3) == "I0")
				imu_msg.header.frame_id = "imu0";
			if(tmp_string.substr(0, 3) == "I1")
			    imu_msg.header.frame_id = "imu1";

			double acc[3];
			double gyro[3];
			size_t pos = 4;

			for(size_t i = 0; i < 3; ++i)
			{
				size_t msg_size = 0;

				while(tmp_string[pos+msg_size] != ',')
				{
					++msg_size;
				}

				gyro[i] = std::stod(tmp_string.substr(pos, msg_size));
				pos += msg_size + 1;
			}

			for(size_t i = 0; i < 3; ++i)
			{
				size_t msg_size = 0;

				while(tmp_string.size() > pos+msg_size && tmp_string[pos+msg_size]!= ',' && tmp_string[pos+msg_size] != '\r')
				{
					++msg_size;
				}

				acc[i] = std::stod(tmp_string.substr(pos, msg_size));
				pos += msg_size + 1;
			}

			imu_msg.linear_acceleration.x = acc[0] / 100.;
			imu_msg.linear_acceleration.y = acc[1] / 100.;
			imu_msg.linear_acceleration.z = acc[2] / 100.;

			// RCLCPP_INFO(this->get_logger(), "acceleration: %f, %f, %f", 
			// imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);

			imu_msg.angular_velocity.x = gyro[0] / 1000.;
			imu_msg.angular_velocity.y = gyro[1] / 1000.;
			imu_msg.angular_velocity.z = gyro[2] / 1000.;

			m_imu_pub->publish(imu_msg);

			// RCLCPP_INFO(this->get_logger(), "received");
		}
		else
		{
			m_buffer.pop();
		}
		
	}
};

auto main(int argc, char ** argv) -> int
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<imu_sender>());
	rclcpp::shutdown();
	
	return 0;
}
