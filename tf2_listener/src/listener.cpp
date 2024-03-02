#include <memory>
#include <array>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class listener_test : public rclcpp::Node
{
	std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
	std::unique_ptr<tf2_ros::TransformListener> m_listener;
	rclcpp::TimerBase::SharedPtr m_timer;

public:
	listener_test() : Node("listener_test")
	{
		m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		m_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
        m_timer = this->create_wall_timer(100ms, std::bind(&listener_test::timer_callback, this));
	}

private:
	void timer_callback()
	{
		geometry_msgs::msg::TransformStamped t;
		try
		{
			t = m_tf_buffer->lookupTransform("world", "pose", tf2::TimePointZero);
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
		}

		RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<listener_test>());
    rclcpp::shutdown();
    return 0;
}
