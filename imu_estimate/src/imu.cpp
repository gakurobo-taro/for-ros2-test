#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/utils.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

using namespace std::chrono_literals;

class imu_estimator : public rclcpp::Node
{
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
	rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_euler_pub;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_imu_reset_sub;
	
	rclcpp::TimerBase::SharedPtr m_timer;

	Eigen::Vector3d m_euler_estimate;
	Eigen::Matrix3d m_P_estimate;

	sensor_msgs::msg::Imu m_imu_msg;

	geometry_msgs::msg::Pose m_imu_pose;

public:
	imu_estimator() : Node("imu_estimator")
	{
		constexpr double dt = 1.0 / 100.0;

		m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
			"imu", 10, std::bind(&imu_estimator::imu_callback, this, std::placeholders::_1));

		m_timer = this->create_wall_timer(10ms, std::bind(&imu_estimator::timer_callback, this));

		m_euler_pub = this->create_publisher<geometry_msgs::msg::Vector3>(
			"imu_euler", 10);

		m_imu_reset_sub = this->create_subscription<std_msgs::msg::Bool>("imu_reset", 10, std::bind(&imu_estimator::imu_reset, this, std::placeholders::_1));

		m_euler_estimate = Eigen::Vector3d::Zero();
		m_P_estimate = 0.0174 * dt * dt * Eigen::Matrix3d::Identity();

		auto param_yaw   = rcl_interfaces::msg::ParameterDescriptor{};
		auto param_pitch = rcl_interfaces::msg::ParameterDescriptor{};
		auto param_roll  = rcl_interfaces::msg::ParameterDescriptor{};

		this->declare_parameter("yaw"  , "0.0", param_yaw);
		this->declare_parameter("pitch", "0.0", param_pitch);
		this->declare_parameter("roll" , "0.0", param_roll);

		auto yaw   = get_parameter("yaw").as_double();
		auto pitch = get_parameter("pitch").as_double();
		auto roll  = get_parameter("roll").as_double();

		tf2::Quaternion q = tf2::Quaternion();
		tf2::convert(tf2::Vector3{yaw, pitch, roll}, q);

		m_imu_pose.orientation.x = q.x();
		m_imu_pose.orientation.y = q.y();
		m_imu_pose.orientation.z = q.z();
		m_imu_pose.orientation.w = q.w();
	}

private:
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		if(msg->header.frame_id == "imu0")
			return;
		m_imu_msg = *msg;
	}

	void timer_callback()
	{
		constexpr double dt = 1. / 100.;

		const double phi = m_euler_estimate(0);
		const double theta = m_euler_estimate(1);
		const double psi = m_euler_estimate(2);

		using namespace std;

		geometry_msgs::msg::Vector3 euler_from_quaternion;

		euler_from_quaternion.x = atan2(2. * (m_imu_pose.orientation.w * m_imu_pose.orientation.x - m_imu_pose.orientation.y * m_imu_pose.orientation.z), 2. * (m_imu_pose.orientation.w * m_imu_pose.orientation.w + m_imu_pose.orientation.z * m_imu_pose.orientation.z) - 1.);
		euler_from_quaternion.y = asin(2. * (m_imu_pose.orientation.w * m_imu_pose.orientation.y + m_imu_pose.orientation.x * m_imu_pose.orientation.z));
		euler_from_quaternion.z = atan2(2. * (m_imu_pose.orientation.w * m_imu_pose.orientation.z - m_imu_pose.orientation.x * m_imu_pose.orientation.y), 2. * (m_imu_pose.orientation.w * m_imu_pose.orientation.w + m_imu_pose.orientation.x * m_imu_pose.orientation.x) - 1.);

		Eigen::Matrix3d rotation_matrix;

		Eigen::Matrix3d rotation_matrix_x;
		Eigen::Matrix3d rotation_matrix_y;
		Eigen::Matrix3d rotation_matrix_z;

		rotation_matrix_x <<
            1, 0, 0,
            0, cos(euler_from_quaternion.x), -sin(euler_from_quaternion.x),
            0, sin(euler_from_quaternion.x), cos(euler_from_quaternion.x);
		
		rotation_matrix_y <<
            cos(euler_from_quaternion.y), 0, sin(euler_from_quaternion.y),
            0, 1, 0,
            -sin(euler_from_quaternion.y), 0, cos(euler_from_quaternion.y);
		
		rotation_matrix_z <<
            cos(euler_from_quaternion.z), -sin(euler_from_quaternion.z), 0,
            sin(euler_from_quaternion.z), cos(euler_from_quaternion.z), 0,
            0, 0, 1;

		rotation_matrix = rotation_matrix_x * rotation_matrix_y * rotation_matrix_z;

		Eigen::Vector3d u;

		Eigen::Vector3d u_pre;

		u_pre <<
            m_imu_msg.angular_velocity.x * dt,
            m_imu_msg.angular_velocity.y * dt,
            m_imu_msg.angular_velocity.z * dt;

		u = rotation_matrix * u_pre;
		
		Eigen::Matrix<double, 2, 3> h;
		h <<
			1, 0, 0,
			0, 1, 0;

	    Eigen::Vector2d z;

		Eigen::Vector3d linear_acceleration;

		linear_acceleration <<
			m_imu_msg.linear_acceleration.x,
            m_imu_msg.linear_acceleration.y,
            m_imu_msg.linear_acceleration.z;

		Eigen::Vector2d fixed_linear_acceleration;

		fixed_linear_acceleration = rotation_matrix * linear_acceleration;

		z <<
			std::atan2(fixed_linear_acceleration(1), fixed_linear_acceleration(2)),
			std::atan2(fixed_linear_acceleration(0), std::hypot(fixed_linear_acceleration(1), fixed_linear_acceleration(3)));
		
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

		geometry_msgs::msg::Vector3 euler_msg;
		euler_msg.set__x(m_euler_estimate.x());
		euler_msg.set__y(m_euler_estimate.y());
		euler_msg.set__z(m_euler_estimate.z());

		m_euler_pub->publish(euler_msg);
	}

	void imu_reset(const std_msgs::msg::Bool::SharedPtr msg)
	{
		if(msg->data)
		{
			m_euler_estimate.x() = 0;
			m_euler_estimate.y() = 0;
			m_euler_estimate.z() = 0;
		}
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<imu_estimator>());
	rclcpp::shutdown();
	return 0;
}
