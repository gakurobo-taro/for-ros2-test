#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>


#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Core>

#include "odom_interface/srv/odom_srv.hpp"

using namespace std::chrono_literals;

class odometry_node : public rclcpp::Node
{
	rclcpp::Client<odom_interface::srv::OdomSrv>::SharedPtr m_odom_client;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_pub;

	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::TimerBase::SharedPtr m_request_timer;

	odom_interface::srv::OdomSrv::Response::SharedPtr m_odom_response;
	
	geometry_msgs::msg::Pose2D::SharedPtr m_odom_pose;	

	struct xy_t
	{
		double x;
		double y;
	};

	/// @brief  unit is mks
	std::array<xy_t, 3> m_odom_wheel_pos = 
		{{
			{0.0, 0.5},
			{-0.25, 0.25},
			{0.25, 0.25}
		}};

public:
	odometry_node() : Node("odometry_node")
	{
		m_odom_client = this->create_client<odom_interface::srv::OdomSrv>("odom_srv");
        m_twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        m_timer = this->create_wall_timer(100ms, std::bind(&odometry_node::timer_callback, this));
		m_request_timer = this->create_wall_timer(100ms, std::bind(&odometry_node::request_timer_callback, this));

		m_odom_pose = std::make_shared<geometry_msgs::msg::Pose2D>();
		m_odom_pose->x = 0.0;
        m_odom_pose->y = 0.0;
		m_odom_pose->theta = 0.0;
	}

private:
    void timer_callback()
	{
#define ESTIMATED 0

#if not ESTIMATED
		constexpr double dt = 1. / 10.;
		bool data_available = (m_odom_response == nullptr);

		if (not data_available)
		{
			RCLCPP_WARN(this->get_logger(), "odom data is not available");
			return;
		}

		Eigen::Vector<double, 6> odom_xy_velocity;

		odom_xy_velocity <<
			m_odom_response->wheel[0] * cos(m_odom_response->steer[0]),
			m_odom_response->wheel[0] * sin(m_odom_response->steer[0]),
            m_odom_response->wheel[1] * cos(m_odom_response->steer[1]),
            m_odom_response->wheel[1] * sin(m_odom_response->steer[1]),	
			m_odom_response->wheel[2] * cos(m_odom_response->steer[2]),
            m_odom_response->wheel[2] * sin(m_odom_response->steer[2]);
		
		Eigen::Matrix<double, 3, 6> kinematics_matrix;

		kinematics_matrix <<
            0.25, 0, 0.25, 0, 0.25, 0,
			0, 0.25, 0, 0.25, 0, 0.25,
			-0.25 * (m_odom_wheel_pos[0].x * cos(m_odom_pose->theta) + m_odom_wheel_pos[0].y * sin(m_odom_pose->theta)),
			 0.25 * (m_odom_wheel_pos[0].x * sin(m_odom_pose->theta) - m_odom_wheel_pos[0].y * cos(m_odom_pose->theta)),
			-0.25 * (m_odom_wheel_pos[1].x * cos(m_odom_pose->theta) + m_odom_wheel_pos[1].y * sin(m_odom_pose->theta)),
			 0.25 * (m_odom_wheel_pos[1].x * sin(m_odom_pose->theta) - m_odom_wheel_pos[1].y * cos(m_odom_pose->theta)),
			-0.25 * (m_odom_wheel_pos[2].x * cos(m_odom_pose->theta) + m_odom_wheel_pos[2].y * sin(m_odom_pose->theta)),
			 0.25 * (m_odom_wheel_pos[2].x * sin(m_odom_pose->theta) - m_odom_wheel_pos[2].y * cos(m_odom_pose->theta));
		

		auto xy_theta_velocity = kinematics_matrix * odom_xy_velocity;

		m_odom_pose->x += xy_theta_velocity(0) * dt;
		m_odom_pose->y += xy_theta_velocity(1) * dt;
		m_odom_pose->theta += xy_theta_velocity(2) * dt;

		auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
		twist_msg->linear.x = xy_theta_velocity(0);
		twist_msg->linear.y = xy_theta_velocity(1);
		twist_msg->angular.z = xy_theta_velocity(2);

		m_twist_pub->publish(*twist_msg);
#endif // not ESTIMATED
	}

	void request_timer_callback()
	{
		if(not m_odom_client->wait_for_service(100ms))
		{
			RCLCPP_WARN(this->get_logger(), "service is not available");
            return;
		}

		for(int i = 0; i < 3; i++)
		{
			auto request = std::make_shared<odom_interface::srv::OdomSrv::Request>();

			request->req = i;

			auto result = m_odom_client->async_send_request(request, std::bind(&odometry_node::response_callback, this, std::placeholders::_1));
		}
	}

	void response_callback(const rclcpp::Client<odom_interface::srv::OdomSrv>::SharedFuture future)
	{
		if(future.get()->available)
			m_odom_response= future.get();
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<odometry_node>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
