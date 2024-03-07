#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/utils.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

using namespace std::chrono_literals;

class imu_estimator : public rclcpp::Node
{
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
	
	rclcpp::TimerBase::SharedPtr m_timer;

	Eigen::Vector3d m_euler_estimate;
	Eigen::Matrix3d m_P_estimate;

	sensor_msgs::msg::Imu m_imu_msg;

public:
	imu_estimator() : Node("imu_estimator")
	{
		constexpr double dt = 1.0 / 4.0;

		m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
			"imu", 10, std::bind(&imu_estimator::imu_callback, this, std::placeholders::_1));

		m_timer = this->create_wall_timer(100ms, std::bind(&imu_estimator::timer_callback, this));

		m_euler_estimate = Eigen::Vector3d::Zero();
		m_P_estimate = 0.0174 * dt * dt * Eigen::Matrix3d::Identity();
	}

private:
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		m_imu_msg = *msg;
	}

	void timer_callback()
	{
		constexpr double dt = 1. / 10.;

		const double phi = m_euler_estimate(0);
		const double theta = m_euler_estimate(1);
		const double psi = m_euler_estimate(2);

		using namespace std;

		geometry_msgs::msg::Vector3 euler_from_quaternion;

		euler_from_quaternion.x = atan2(2 * (m_imu_msg.orientation.w * m_imu_msg.orientation.x - m_imu_msg.orientation.y * m_imu_msg.orientation.z), 2 * (m_imu_msg.orientation.w * m_imu_msg.orientation.w + m_imu_msg.orientation.z * m_imu_msg.orientation.z) - 1);
		euler_from_quaternion.y = asin(2 * (m_imu_msg.orientation.w * m_imu_msg.orientation.y + m_imu_msg.orientation.x * m_imu_msg.orientation.z));
		euler_from_quaternion.z = atan2(2 * (m_imu_msg.orientation.w * m_imu_msg.orientation.z - m_imu_msg.orientation.x * m_imu_msg.orientation.y), 2 * (m_imu_msg.orientation.w * m_imu_msg.orientation.w + m_imu_msg.orientation.x * m_imu_msg.orientation.x) - 1);

		Eigen::Vector3d u;

		u <<
		    m_imu_msg.angular_velocity.x * dt,
			m_imu_msg.angular_velocity.y * dt,
        	m_imu_msg.angular_velocity.z * dt;
		
		Eigen::Matrix<double, 2, 3> h;
		h <<
			1, 0, 0,
			0, 1, 0;

	    Eigen::Vector2d z;

		z <<
			std::atan2(m_imu_msg.linear_acceleration.y, m_imu_msg.linear_acceleration.z),
			std::atan2(m_imu_msg.linear_acceleration.x, std::hypot(m_imu_msg.linear_acceleration.y, m_imu_msg.linear_acceleration.z));
		
		Eigen::Matrix3d Q;

		Q <<
			0.0174 * dt*dt, 0, 0,
			0, 0.0174 * dt*dt, 0,
            0, 0, 0.0174 * dt*dt;
		
		Eigen::Matrix2d R;
		R << 
			dt * dt, 0,
			0, dt * dt;

		Eigen::Matrix3d F_k;

		F_k <<
			1 + u(1) * cos(phi) * tan(theta) - u(2) * sin(phi) * tan(theta), u(1) * sin(phi) / cos(theta) / cos(theta) + u(2) * cos(phi) / cos(theta) /  cos(theta), 0,
			-u(1) * sin(phi) - u(2) * cos(phi), 1, 0,
			u(1) * cos(phi) / cos(theta) - u(2) * sin(phi) / cos(theta), u(1) * sin(phi) * tan(theta) / cos(theta) + u(2) * cos(phi) * tan(theta) / cos(theta), 1;

		Eigen::Vector3d prediction_euler;

		prediction_euler <<
			phi + u(0) + u(1) * sin(phi) * tan(theta) + u(2) * cos(phi) * tan(theta),
			theta + u(1) * cos(phi) - u(2) * sin(phi),
			psi + u(1) * sin(phi) / cos(theta) + u(2) * cos(phi) / cos(theta);
		
		Eigen::Matrix3d prediction_P;

		prediction_P = F_k * m_P_estimate * F_k.transpose() + Q;

		Eigen::Vector2d y;

		y = z - h * prediction_euler;
		
		Eigen::Matrix2d S;

        S = h * prediction_P * h.transpose() + R;

		Eigen::Matrix<double, 3, 2> K;

		prediction_P * h.transpose() * S.inverse();

		m_euler_estimate = prediction_euler + K * y;

		m_P_estimate = (Eigen::Matrix<double, 3, 3>::Identity() - K * h) * prediction_P;
		RCLCPP_INFO(this->get_logger(), "prediction: %f %f %f", prediction_euler(0), prediction_euler(1), prediction_euler(2));
		RCLCPP_INFO(this->get_logger(), "z: %f %f", z(0), z(1));
		RCLCPP_INFO(this->get_logger(), "estimate: %f %f %f", m_euler_estimate(0), m_euler_estimate(1), m_euler_estimate(2));
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<imu_estimator>());
	rclcpp::shutdown();
	return 0;
}
