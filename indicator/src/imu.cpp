#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class imu_node : public rclcpp::Node
{
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr m_imu_sub;
	rclcpp::TimerBase::SharedPtr m_timer;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;

	std::string m_imu_msg;

public:
	imu_node() : Node("imu_node")
	{
		m_imu_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
			"uart_rx_msg", 10, std::bind(&imu_node::imu_callback, this, std::placeholders::_1));
		m_timer = this->create_wall_timer(10ms, std::bind(&imu_node::timer_callback, this));
		m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
	}

private:
	void imu_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
	{
		if(msg->data[0] == '1')
		{
			return;
		}
		std::string s_msg;

		for(auto s : msg->data)
		{
			s_msg += (char)s;
		}

		static std::string buffer;

		buffer += s_msg;

		for(int i = 0; i < buffer.size(); i++)
		{
			if(buffer[i] == '\n')
			{
				size_t comma_counter = 0;

				for(auto c : buffer)
				{
					if(c == ',')
                    {
                        comma_counter++;
                    }
				}

				if(comma_counter != 13)
				{
					//RCLCPP_WARN(this->get_logger(), "Invalid imu message");
					buffer.clear();
					return;
				}

				m_imu_msg = buffer;
				buffer.clear();
			}
		}

		
	}

	void timer_callback()
	{
		if(m_imu_msg.size() == 0)
		    return;
		
		std::vector<std::string> imu_data{""};
		{
		size_t pos = 0;
		for(auto c : m_imu_msg)
		{
			if(c == ',')
			{
				pos++;

				imu_data.push_back("");

				continue;
			}

			imu_data[pos] += c;
		}}

		std::vector<int> imu_data_int;

		for(auto s : imu_data)
		{
			imu_data_int.push_back(std::stoi(s));
		}

		

		sensor_msgs::msg::Imu imu_msg;
		imu_msg.header.stamp = this->now();
		imu_msg.header.frame_id = "imu";
		imu_msg.orientation.x = (double)imu_data_int[1] / pow(2., 14.);
		imu_msg.orientation.y = (double)imu_data_int[2] / pow(2., 14.);
		imu_msg.orientation.z = (double)imu_data_int[3] / pow(2., 14.);
		imu_msg.orientation.w = (double)imu_data_int[4] / pow(2., 14.);
		imu_msg.linear_acceleration.x = (double)imu_data_int[8] / 100.;
		imu_msg.linear_acceleration.y = (double)imu_data_int[9] / 100.;
		imu_msg.linear_acceleration.z = (double)imu_data_int[10] / 100.;
		imu_msg.angular_velocity.x = (double)imu_data_int[5] / 900.;
		imu_msg.angular_velocity.y = (double)imu_data_int[6] / 900.;
		imu_msg.angular_velocity.z = (double)imu_data_int[7] / 900.;

		m_imu_pub->publish(imu_msg);

		RCLCPP_INFO(this->get_logger(), "imu_data: %f, %f, %f", imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
		m_imu_msg.clear();
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<imu_node>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
