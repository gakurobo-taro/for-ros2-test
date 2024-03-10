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

inline static const std::array<uint32_t, 3> wheel_ids {0x01200100, 0x01200000, 0x01200200};
inline static const std::array<uint32_t, 3> steer_ids {0x01210100, 0x01210000, 0x01210200};

class odom_server : public rclcpp::Node
{
	rclcpp::Service<odom_interface::srv::OdomSrv>::SharedPtr m_odom_service;
	rclcpp::Subscription<can_msgs::msg::CanMsg>::SharedPtr m_can_subscriber;

	odom_interface::srv::OdomSrv::Response m_odom_srv_response;
	std::bitset<6> m_odom_srv_response_ready = false;

	std::vector<uint32_t> m_ids;

public:
	odom_server() : Node("odom_server")
	{
		m_odom_service = this->create_service<odom_interface::srv::OdomSrv>(
            "odom_service",
            std::bind(&odom_server::odom_callback, this, std::placeholders::_1, std::placeholders::_2));

        m_can_subscriber = this->create_subscription<can_msgs::msg::CanMsg>(
            "can_subscriber", 10, std::bind(&odom_server::can_callback, this, std::placeholders::_1));

		constexpr uint32_t spd = 0x20;
		constexpr uint32_t pos = 0x30;

		std::vector<uint32_t> ids;

		for(auto &id : wheel_ids)
		{
			ids.push_back(id | spd);
		}

		for(auto &id : steer_ids)
		{
			ids.push_back(id | pos);
		}
	}

private:
    void odom_callback(const odom_interface::srv::OdomSrv::Request::SharedPtr request, odom_interface::srv::OdomSrv::Response::SharedPtr response)
	{
		if(not request->req)
		{
			response->available = false;
            return;
		}

		if(not m_odom_srv_response_ready.all())
		{
			RCLCPP_WARN(this->get_logger(), "odom_srv_response not ready yet");
			return;
		}

		auto res = std::make_shared<odom_interface::srv::OdomSrv::Response>(m_odom_srv_response);

		response = res;
	}

	void can_callback(const can_msgs::msg::CanMsg::SharedPtr msg)
	{
		int ret = 0;
		for(uint i = 0; i < m_ids.size(); i++)
		{
			if(msg->id == m_ids[i])
			{
				ret = i;
			}
		}

		if(ret == 0)
		{
			return;
		}
		
		m_odom_srv_response_ready.set(ret, true);

		uint32_t tmp = 0;

		for(int i = 0; i < 4; i++)
		{
			tmp += (msg->data[i] << (24 - (i * 8)));
		}

		if(ret < 3)
		{
			m_odom_srv_response.wheel[ret] = std::bit_cast<float>(tmp);
		}

		if(ret >= 3)
		{
			m_odom_srv_response.steer[ret-3] = std::bit_cast<float>(tmp);
		}
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<odom_server>());
	rclcpp::shutdown();
	return 0;
}
