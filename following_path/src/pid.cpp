#include <memory>
#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


geometry_msgs::msg::Vector3 operator+(const geometry_msgs::msg::Vector3& lhs, const geometry_msgs::msg::Vector3& rhs)
{
	geometry_msgs::msg::Vector3 result;
	result.x = lhs.x + rhs.x;
	result.y = lhs.y + rhs.y;
	result.z = lhs.z + rhs.z;
	return result;
}

geometry_msgs::msg::Vector3 operator-(const geometry_msgs::msg::Vector3& lhs, const geometry_msgs::msg::Vector3& rhs)
{
	geometry_msgs::msg::Vector3 result;
	result.x = lhs.x - rhs.x;
	result.y = lhs.y - rhs.y;
	result.z = lhs.z - rhs.z;
	return result;
}

geometry_msgs::msg::Vector3 operator-(const geometry_msgs::msg::Vector3& lhs)
{
	geometry_msgs::msg::Vector3 result;
	result.x = -lhs.x;
	result.y = -lhs.y;
	result.z = -lhs.z;
	return result; 
}

template<typename T>
requires std::is_arithmetic_v<T>
geometry_msgs::msg::Vector3 operator*(const geometry_msgs::msg::Vector3 &lhs, T arithmetic)
{
	geometry_msgs::msg::Vector3 result;
    result.x = lhs.x * arithmetic;
    result.y = lhs.y * arithmetic;
    result.z = lhs.z * arithmetic;
    return result;
}

geometry_msgs::msg::Vector3 operator+=(geometry_msgs::msg::Vector3& lhs, const geometry_msgs::msg::Vector3 rhs)
{
	lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

geometry_msgs::msg::Twist operator+=(geometry_msgs::msg::Twist &hs, const geometry_msgs::msg::Twist &rhs)
{
	hs.linear += rhs.linear;
    hs.angular += rhs.angular;
    return hs;
}

using namespace std::chrono_literals;

class pid : public rclcpp::Node
{
	std::unique_ptr<tf2_ros::Buffer> m_buffer;
	std::unique_ptr<tf2_ros::TransformListener> m_listener;

	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;

public:
	pid(const std::string& node_name = "tf_pid") : Node(node_name)
	{
		m_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_listener = std::make_unique<tf2_ros::TransformListener>(*m_buffer);

        m_timer = this->create_wall_timer(100ms, std::bind(&pid::timer_callback, this));
		m_pub = this->create_publisher<geometry_msgs::msg::Twist>("vel_cmd", 10);
	}

private:
	void timer_callback()
	{
		geometry_msgs::msg::TransformStamped transform;
		try
		{
			transform = m_buffer->lookupTransform("world", "pose", tf2::TimePointZero);
		}
		catch (tf2::TransformException& ex)
		{
			RCLCPP_WARN(this->get_logger(), "%s", ex.what());
			return;
		}

		geometry_msgs::msg::Twist twist;

		double p_gain = 0.01;
		double i_gain = 0.01;
		double d_gain = 0.01;

		static geometry_msgs::msg::TransformStamped last_transform;

		geometry_msgs::msg::Vector3 linear_v;
		geometry_msgs::msg::Vector3 angular_v;

		linear_v = transform.transform.translation;

		tf2::Quaternion q(
			transform.transform.rotation.w,
			transform.transform.rotation.x,
			transform.transform.rotation.y,
			transform.transform.rotation.z);
		tf2::Matrix3x3 m(q);

		m.getRPY(angular_v.x, angular_v.y, angular_v.z);

		static geometry_msgs::msg::Twist twist_sigma;

		geometry_msgs::msg::Vector3 diff_linear_v;
		geometry_msgs::msg::Vector3 diff_angular_v;

		geometry_msgs::msg::Vector3 last_angular_v;

		diff_linear_v = linear_v - last_transform.transform.translation;

		tf2::Quaternion q_last(
			last_transform.transform.rotation.w,
            last_transform.transform.rotation.x,
            last_transform.transform.rotation.y,
            last_transform.transform.rotation.z);
		tf2::Matrix3x3 m_last(q_last);

		m_last.getRPY(last_angular_v.x, last_angular_v.y, last_angular_v.z);
		diff_angular_v = angular_v - last_angular_v;


		twist.linear = 
			linear_v * p_gain 
		  + twist_sigma.linear * i_gain 
		  - diff_linear_v * d_gain;
		
		twist.angular =
		    angular_v * p_gain
		  + twist_sigma.angular * i_gain
		  - diff_angular_v * d_gain;

		twist_sigma += twist;
		last_transform = transform;

		m_pub->publish(twist);

		RCLCPP_INFO(this->get_logger(), "linear: x:%f y:%f z:%f, angular: x:%f y:%f z:%f", linear_v.x, linear_v.y, linear_v.z, angular_v.x, angular_v.y, angular_v.z);
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<pid>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
