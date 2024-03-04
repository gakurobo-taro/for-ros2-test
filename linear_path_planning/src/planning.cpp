#include <memory>
#include <array>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

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

double l2_norm(const geometry_msgs::msg::Vector3& vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

void Vector3_set(geometry_msgs::msg::Vector3& vector, const double value)
{
	vector.x = value;
    vector.y = value;
    vector.z = value;
}

class planning : public rclcpp::Node
{
	std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;

	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_pose_sub;

	geometry_msgs::msg::Pose::SharedPtr m_pose;

	bool planed = false;

public:
	planning() : Node("planning")
	{
		m_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
			"target_pose", 10, std::bind(&planning::pose_callback, this, std::placeholders::_1));
		
		m_pose = nullptr;

		m_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		m_timer = this->create_wall_timer(100ms, std::bind(&planning::timer_callback, this));
	}

private:
	void timer_callback()
	{
		if(not m_pose)
		{
			RCLCPP_WARN(this->get_logger(), "No target pose");
			return;
		}

		static std::vector<geometry_msgs::msg::Pose> targets;

		if(not planed)
		{
			plan();
		}
		
		
	}

	void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
	{
		m_pose = msg;
	}

	std::vector<geometry_msgs::msg::Pose> plan()
	{
		using namespace geometry_msgs::msg;
		static Twist now_vel;
		static Accel now_accel;

		constexpr double dt = 0.01;
		constexpr double acceleration_limit = 10; // tekitou
		constexpr double speed_limit = 2;

		// check can stop
		Vector3 pos_vector;
		pos_vector.x = m_pose->position.x;
		pos_vector.y = m_pose->position.y;
		pos_vector.z = m_pose->position.z;

		std::vector<Pose> targets;
		
		bool stop = false;

		while (not stop)
		{
			static size_t counter = 0;
			counter++;

			if(counter > 100000)
			{
				RCLCPP_WARN(this->get_logger(), "Too many iterations");
                stop = true;
			}
			static Vector3 pos_vector_now;
			static Twist vel_now;

			if(l2_norm(pos_vector - pos_vector_now) < 0.01)
			{
				stop = true;
				Vector3_set(pos_vector_now, 0.);
			}

			double acceleration;
			double now_spd = l2_norm(vel_now.linear);

			if(2.*acceleration_limit * l2_norm(pos_vector - pos_vector_now) > now_spd * now_spd)
			{
				acceleration = acceleration_limit;
			} 
			else
			{
				acceleration = -acceleration_limit;
			}
			
			double distance = l2_norm(pos_vector_now) + now_spd * dt;

			double spd = now_spd + acceleration * dt;

			if(spd > speed_limit)
			{
				spd = speed_limit;
			}
			
			Vector3 unit_vector = (pos_vector - pos_vector_now) * (1. / l2_norm(pos_vector - pos_vector_now));

			pos_vector_now = unit_vector * distance;
			vel_now.linear = unit_vector * spd;

			Pose pose;
			pose.position.x = pos_vector_now.x;
			pose.position.y = pos_vector_now.y;
			pose.position.z = pos_vector_now.z;

			targets.push_back(pose);

			RCLCPP_INFO(this->get_logger(), "spd %lf", unit_vector.x);
		}

		// for(auto& target : targets)
		// {
		// 	RCLCPP_INFO(this->get_logger(), "target: %f %f %f", target.position.x, target.position.y, target.position.z);
		// }
		// RCLCPP_INFO(this->get_logger(), "iterations: %d", targets.size());
		planed = true;
		return targets;
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<planning>());
	rclcpp::shutdown();
	return 0;
}
