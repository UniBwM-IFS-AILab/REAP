/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief vehicle uORB topic listener
 * @file vehicle_status_listener_lib.cpp
 * @addtogroup examples
 * @author Lucas Mair
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

/**
 * @brief Sensor Combined uORB topic data callback
 */
class VehicleStatusListener : public rclcpp::Node
{
public:
	
	px4_msgs::msg::BatteryStatus::SharedPtr recent_battery_msg;
	px4_msgs::msg::VehicleStatus::SharedPtr recent_status_msg;
	
	// name_prefix should have the format "<identifier>/"
	explicit VehicleStatusListener(std::string name_prefix = "") : Node(name_prefix.substr(0, name_prefix.size() - 1)+ "_" + "vehicle_status_listener") {
		
		px4_msgs::msg::BatteryStatus empty_bat_msg{};
		recent_battery_msg = std::make_shared<px4_msgs::msg::BatteryStatus>(std::move(empty_bat_msg));
		
		px4_msgs::msg::VehicleStatus empty_stat_msg{};
		recent_status_msg = std::make_shared<px4_msgs::msg::VehicleStatus>(std::move(empty_stat_msg));

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		battery_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
			name_prefix + "fmu/out/battery_status", qos,
			[this](const px4_msgs::msg::BatteryStatus::UniquePtr msg) {
				/*
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED BATTERY DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
			//std::cout << "discharged_mah: " << msg->discharged_mah  << std::endl; <-- permanent 0
			std::cout << "remaining battery percentage: " << 100*(msg->remaining) << "%" << std::endl;
			//std::cout << "full_charge_capacity_wh: " << msg->full_charge_capacity_wh  << std::endl; <-- permanent 0
			//std::cout << "remaining_capacity_wh: " << msg->remaining_capacity_wh  << std::endl; <-- permanent 0
			*/
			
			//recent_battery_msg = std::move(msg); <-- this doesnt work because the unique_ptr is const
			
			recent_battery_msg = std::make_unique<px4_msgs::msg::BatteryStatus>(*msg);
			//std::cout << "recent_battery_msg timestamp (should be same as above): " << recent_battery_msg->timestamp << std::endl;

		});
		
		vehicle_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			name_prefix + "fmu/out/vehicle_status", qos,
			[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
			/*
			https://docs.px4.io/main/en/msg_docs/VehicleStatus.html
			
			uint8 arming_state
			uint8 ARMING_STATE_DISARMED = 1
			uint8 ARMING_STATE_ARMED    = 2
			
			uint8 nav_state
			uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
			uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
			*/
			
			recent_status_msg = std::make_unique<px4_msgs::msg::VehicleStatus>(*msg);
			//std::cout << "recent_status_msg timestamp (should be same as above): " << recent_status_msg->timestamp << std::endl;

		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_subscription_;
};

/*
int main(int argc, char *argv[]) {
	std::cout << "Starting vehicle_status listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleStatusListener>());
	

	rclcpp::shutdown();
	return 0;
}*/
