#include <memory>
#include <chrono>
#include <vector>
#include <array>
#include <string>
#include <bit>
#include <bitset>

#include <rclcpp/rclcpp.hpp>

#include "odom_interface/srv/odom_srv.hpp"
#include "can_msgs/msg/can_msg.hpp"

inline static const std::array<uint32_t, 3> wheel_ids {0x00200100, 0x00200000, 0x00200200};
inline static const std::array<uint32_t, 3> steer_ids {0x00210100, 0x00210000, 0x00210200};

class odom_server : public rclcpp::Node
{
	rclcpp::Service<odom_interface::srv::OdomSrv>::SharedPtr m_odom_service;
	rclcpp::Subscription<can_msgs::msg::CanMsg>::SharedPtr m_can_subscriber;

	odom_interface::srv::OdomSrv::Response m_odom_srv_response;
	std::bitset<6> m_odom_srv_response_ready = 0;

	std::vector<uint32_t> m_ids;

public:
	odom_server() : Node("odom_server")
	{
		m_odom_service = this->create_service<odom_interface::srv::OdomSrv>(
            "odom_service",
            std::bind(&odom_server::odom_callback, this, std::placeholders::_1, std::placeholders::_2));

        m_can_subscriber = this->create_subscription<can_msgs::msg::CanMsg>(
            "slcan_recv", 10, std::bind(&odom_server::can_callback, this, std::placeholders::_1));

		static const constinit uint32_t spd = 0x20;
		static const constinit uint32_t pos = 0x30;

		std::vector<uint32_t> ids;

		for(auto &id : wheel_ids)
		{
			ids.push_back(id | spd);
		}

		for(auto &id : steer_ids)
		{
			ids.push_back(id | pos);
		}

		m_ids = std::move(ids);
	}

private:
	/**
	 * @brief ros2 service callback function.
	 * You can't call this function!!!
	 * 
	 * @param request 
	 * @param response 
	 * @return odom_interface::srv::OdomSrv::Response::SharedPtr 
	 */
    odom_interface::srv::OdomSrv::Response::SharedPtr odom_callback(
		const odom_interface::srv::OdomSrv::Request::SharedPtr request, 
		odom_interface::srv::OdomSrv::Response::SharedPtr response)
	{
		if(not request->req)
		{
			response->available = false;
			RCLCPP_WARN(this->get_logger(), "odom_service request not available");
            return nullptr;
		}

		if(not m_odom_srv_response_ready.all())
		{
			response->available = false;
			RCLCPP_WARN(this->get_logger(), "odom_srv_response not ready yet");
			return nullptr;
		}

		*response = m_odom_srv_response;

		response->available = true;

		RCLCPP_INFO(this->get_logger(), "response");

		return response;
	}

	/**
	 * @brief sub callback function
	 * 
	 * @param msg 
	 */
	void can_callback(const can_msgs::msg::CanMsg::SharedPtr msg)
	{
		int ret = 0;
		bool is_truth_id = false;

		auto check_id = msg->id & ~0x0F000000;

		for(uint i = 0; i < m_ids.size(); i++)
		{
			if(check_id == m_ids[i])
			{
				is_truth_id = true;
				ret = i;
			}
		}

		if(not is_truth_id)
		{
			return;
		}
		
		m_odom_srv_response_ready.set(ret, true);

		uint32_t tmp = 0;

		for(int i = 0; i < 4; i++)
		{
			tmp += (msg->data[i] << ((i * 8)));
		}

		if(ret < 3)
		{
			m_odom_srv_response.wheel[ret] = std::bit_cast<float>(tmp);
		}

		if(ret >= 3)
		{
			m_odom_srv_response.steer[ret-3] = std::bit_cast<float>(tmp);
		}

		RCLCPP_INFO(this->get_logger(), "odom_srv_response ready: %s", m_odom_srv_response_ready.to_string().c_str());
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<odom_server>());
	rclcpp::shutdown();
	return 0;
}
