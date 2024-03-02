#include <memory>
#include <array>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class broadcast_test : public rclcpp::Node
{
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;

	rclcpp::TimerBase::SharedPtr timer;


public:
	broadcast_test(const std::string node_name = "broadcast_test") : Node(node_name)
	{
		pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"pose", 10, std::bind(&broadcast_test::PoseCallback, this, std::placeholders::_1));
		
		timer = this->create_wall_timer(100ms, std::bind(&broadcast_test::timer_callback, this));
		
		tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	}

private:
	void PoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
	{
		geometry_msgs::msg::TransformStamped t;
		geometry_msgs::msg::TransformStamped t2;
		geometry_msgs::msg::TransformStamped t3;

		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = "pose";
		t.child_frame_id = "target";

		tf2::Quaternion q;

		q.setRPY(0., 0., msg->pose.orientation.z);

		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();

		tf_broadcaster->sendTransform(t);
		tf_broadcaster->sendTransform(t2);
	}

	void timer_callback()
	{
		geometry_msgs::msg::TransformStamped t;
		geometry_msgs::msg::TransformStamped t2;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "pose";

		t2.header.stamp = this->get_clock()->now();
		t2.header.frame_id = "pose";
		t2.child_frame_id = "target";

		static int i = 0;

        t.transform.translation.x = i;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

		++i;

        tf2::Quaternion q;

        q.setRPY(0., 0., 0.);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster->sendTransform(t);
		tf_broadcaster->sendTransform(t2);
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<broadcast_test>());
	rclcpp::shutdown();
	
	return 0;
}
