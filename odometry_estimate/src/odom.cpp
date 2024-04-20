#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Core>

#include "odom_interface/srv/odom_srv.hpp"

using namespace std::chrono_literals;

class odometry_node : public rclcpp::Node
{
	rclcpp::Client<odom_interface::srv::OdomSrv>::SharedPtr m_odom_client;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_pub;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_reset_sub;
	rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr m_set_pose_sub;

	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::TimerBase::SharedPtr m_request_timer;

	odom_interface::srv::OdomSrv::Response::SharedPtr m_odom_response;
	
	geometry_msgs::msg::Pose2D::SharedPtr m_odom_pose;

	std::unique_ptr<tf2_ros::TransformBroadcaster> m_odom_tf_broadcaster;

	struct xy_t
	{
		double x;
		double y;
	};

	/// @brief  unit is mks
	constexpr static std::array<xy_t, 3> m_odom_wheel_pos = 
		{{
			{ -0.245,  0.1033333},
			{  0.245,  0.1033333},
			{  0.0  , -0.2066667},
			
		}};
	constexpr static std::array<double, 3> m_odom_steer_radius = 
		{
			std::hypot( -0.245,  0.1033333),
			std::hypot(  0.245,  0.1033333),
			std::hypot(  0.0  , -0.2066667),
		};
	constexpr static std::array<double, 3> m_odom_steer_theta = 
		{
			std::atan2( -0.245,  0.1033333),
			std::atan2(  0.245,  0.1033333),
			std::atan2(  0.0  , -0.2066667),
		};

public:
	odometry_node() : Node("odometry_node")
	{
		m_odom_client = this->create_client<odom_interface::srv::OdomSrv>("odom_service");
        m_twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("odom_velocity", 10);
		m_reset_sub = this->create_subscription<std_msgs::msg::Bool>(
			"odom_reset", 10, std::bind(&odometry_node::reset_pose, this, std::placeholders::_1));
		m_set_pose_sub = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "odom_set_pose", 10, std::bind(&odometry_node::set_pose, this, std::placeholders::_1));

        m_timer = this->create_wall_timer(50ms, std::bind(&odometry_node::timer_callback, this));
		m_request_timer = this->create_wall_timer(10ms, std::bind(&odometry_node::request_timer_callback, this));

		m_odom_pose = std::make_shared<geometry_msgs::msg::Pose2D>();
		m_odom_pose->x = 0.0;
		m_odom_pose->y = 0.0;
		m_odom_pose->theta = 0.0;

		m_odom_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		m_odom_response = nullptr;
	}

private:
    void timer_callback()
	{
#define ESTIMATED 0

#if not ESTIMATED
		constexpr double dt = 1. / 20.;

		if (not m_odom_response)
		{
			RCLCPP_WARN(this->get_logger(), "odom data is nullptr");
			return ;
		}

		bool data_available = (bool)m_odom_response->available;

		if (not data_available)
		{
			RCLCPP_WARN(this->get_logger(), "odom data is not available");
			return;
		}

		// RCLCPP_INFO(this->get_logger(), "odom data: steer:{%lf, %lf, %lf}, wheel:{%lf, %lf, %lf}", 
		// 	m_odom_response->steer[0], m_odom_response->steer[1], m_odom_response->steer[2], 
		// 	m_odom_response->wheel[0], m_odom_response->wheel[1], m_odom_response->wheel[2]);

		Eigen::Vector<double, 6> odom_xy_velocity;

		constexpr double wheel_radius = 0.092 / 2.;
		constexpr double coeff = wheel_radius;

		odom_xy_velocity <<
			m_odom_response->wheel[0] * -sin(m_odom_response->steer[0]) * coeff,
			m_odom_response->wheel[0] *  cos(m_odom_response->steer[0]) * coeff,
            m_odom_response->wheel[1] * -sin(m_odom_response->steer[1]) * coeff,
            m_odom_response->wheel[1] *  cos(m_odom_response->steer[1]) * coeff,
			m_odom_response->wheel[2] * -sin(m_odom_response->steer[2]) * coeff,
        	m_odom_response->wheel[2] *  cos(m_odom_response->steer[2]) * coeff;
		
		Eigen::Matrix<double, 3, 6> kinematics_matrix;

		static constinit const double div = 1.0 / 3.0;
		RCLCPP_INFO(this->get_logger(), "wheel: {%5lf, %5lf, %5lf}", 
			std::hypot(odom_xy_velocity(0), odom_xy_velocity(1)), 
			std::hypot(odom_xy_velocity(2), odom_xy_velocity(3)), 
			std::hypot(odom_xy_velocity(4), odom_xy_velocity(5)));
		RCLCPP_INFO(this->get_logger(), "abs_enc: {%5lf, %5lf, %5lf}", m_odom_response->steer[0], m_odom_response->steer[1], m_odom_response->steer[2]);
		RCLCPP_INFO(this->get_logger(), "odom_xy_velocity: w1: {%5lf, %5lf}, w2: {%5lf, %5lf}, w3: {%5lf, %5lf}",
			odom_xy_velocity(0), odom_xy_velocity(1), odom_xy_velocity(2),
			odom_xy_velocity(3), odom_xy_velocity(4), odom_xy_velocity(5));

		kinematics_matrix <<
            div, 0., div, 0., div, 0.,
			0., div, 0., div, 0., div,
			div *  std::sin(this->m_odom_steer_theta[0]) / m_odom_steer_radius[0],
			div * -std::cos(this->m_odom_steer_theta[0]) / m_odom_steer_radius[0],
			div *  std::sin(this->m_odom_steer_theta[1]) / m_odom_steer_radius[1],
			div * -std::cos(this->m_odom_steer_theta[1]) / m_odom_steer_radius[1],
			div *  std::sin(this->m_odom_steer_theta[2]) / m_odom_steer_radius[2],
			div * -std::cos(this->m_odom_steer_theta[2]) / m_odom_steer_radius[2];
		

		auto xy_theta_velocity = kinematics_matrix * odom_xy_velocity;

		Eigen::Matrix3d rotation_matrix;
		rotation_matrix <<
			cos(m_odom_pose->theta), -sin(m_odom_pose->theta), 0,
			sin(m_odom_pose->theta),  cos(m_odom_pose->theta), 0,
			0                      , 0                       , 1;

		auto truth_velocity = rotation_matrix * xy_theta_velocity;

		m_odom_pose->x +=xy_theta_velocity(0) * dt;
		m_odom_pose->y += xy_theta_velocity(1) * dt;
		// m_odom_pose->theta += xy_theta_velocity(2) * dt;

		auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
		twist_msg->linear.x = xy_theta_velocity(0);
		twist_msg->linear.y = xy_theta_velocity(1);
		twist_msg->angular.z = xy_theta_velocity(2);
		
		RCLCPP_INFO(this->get_logger(), "vx: %5lf, vy: %5lf, vz: %5lf", xy_theta_velocity(0), xy_theta_velocity(1), xy_theta_velocity(2));
		RCLCPP_INFO(this->get_logger(), "tvx: %5lf, tvy: %5lf, tvz: %5lf", truth_velocity(0), truth_velocity(1), truth_velocity(2));
		RCLCPP_INFO(this->get_logger(), "x: %5lf, y: %5lf, theta: %5lf", m_odom_pose->x, m_odom_pose->y, m_odom_pose->theta);
		m_twist_pub->publish(*twist_msg);

		geometry_msgs::msg::TransformStamped tf_msg;
		tf_msg.header.stamp = this->now();
		tf_msg.header.frame_id = "odometry";
		tf_msg.child_frame_id = "base_footprint";

		tf_msg.transform.translation.x = m_odom_pose->x;
		tf_msg.transform.translation.y = m_odom_pose->y;
		tf_msg.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0, 0, m_odom_pose->theta);

		tf_msg.transform.rotation.x = q.getX();
		tf_msg.transform.rotation.y = q.getY();
		tf_msg.transform.rotation.z = q.getZ();
		tf_msg.transform.rotation.w = q.getW();

		m_odom_tf_broadcaster->sendTransform(tf_msg);

		m_odom_response = nullptr;
#endif // not ESTIMATED
	}

	void request_timer_callback()
	{
		if(not m_odom_client->wait_for_service(100ms))
		{
			RCLCPP_WARN(this->get_logger(), "service is not available");
            return;
		}

		auto request = std::make_shared<odom_interface::srv::OdomSrv::Request>();

		request->req = true;

		auto result = m_odom_client->async_send_request(request, std::bind(&odometry_node::response_callback, this, std::placeholders::_1));
		
	}

	void response_callback(const rclcpp::Client<odom_interface::srv::OdomSrv>::SharedFuture future)
	{
		std::future_status status = future.wait_for(1ms);

		if(status == std::future_status::timeout)
		{
			RCLCPP_INFO(this->get_logger(), "service is timeout");
			return;
		}

		if(status == std::future_status::deferred)
		{
			RCLCPP_INFO(this->get_logger(), "service is deferred");
			return;
		}

		if(future.get()->available)
		{
			// RCLCPP_INFO(this->get_logger(), "response");
			m_odom_response = future.get();
		}
			
	}

	void reset_pose(std_msgs::msg::Bool::SharedPtr msg)
	{
		if(msg->data)
		{
			m_odom_pose->x = 0.0;
            m_odom_pose->y = 0.0;
            m_odom_pose->theta = 0.0;
		}
	}

	void set_pose(const geometry_msgs::msg::Pose2D::SharedPtr msg)
	{
		m_odom_pose->x = msg->x;
        m_odom_pose->y = msg->y;
        m_odom_pose->theta = msg->theta;
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
