/****************************************************************************
 *
 * Copyright 2019 PX4 Development Team. All rights reserved.
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
 * @brief Vehicle global position uORB topic listener example
 * @file vehicle_global_position_listener_lib.cpp
 * @addtogroup lib
 * @author Lucas Mair <lucas.mair@unibw.de>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/home_position.hpp>

/**
 * @brief Vehicle global position uORB topic data callback
 */
class VehicleGlobalPositionListener : public rclcpp::Node
{
public:

	px4_msgs::msg::VehicleGlobalPosition::SharedPtr recent_gps_msg;
	px4_msgs::msg::VehicleOdometry::SharedPtr recent_ned_msg;
	px4_msgs::msg::HomePosition::SharedPtr recent_home_msg;
	
	// name_prefix should have the format "<identifier>/"
	explicit VehicleGlobalPositionListener(std::string name_prefix = "") : Node(name_prefix.substr(0, name_prefix.size() - 1)+ "_" + "vehicle_global_position_listener") {
		
		px4_msgs::msg::VehicleGlobalPosition empty_gps_msg{};
		recent_gps_msg = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>(std::move(empty_gps_msg));
		
		px4_msgs::msg::VehicleOdometry empty_ned_msg{};
		recent_ned_msg = std::make_shared<px4_msgs::msg::VehicleOdometry>(std::move(empty_ned_msg));
		
		px4_msgs::msg::HomePosition empty_home_msg{};
		recent_home_msg = std::make_shared<px4_msgs::msg::HomePosition>(std::move(empty_home_msg));

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		subscription_gps = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
			name_prefix + "fmu/out/vehicle_global_position", qos,
			[this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
			/*
			std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE GLOBAL POSITION DATA"   << std::endl;
			std::cout << "=================================="   << std::endl;
			std::cout << "ts: "      << msg->timestamp    << std::endl;
			std::cout << "lat: " << msg->lat  << std::endl;
			std::cout << "lon: " << msg->lon << std::endl;
			std::cout << "alt: " << msg->alt  << std::endl;
			std::cout << "alt_ellipsoid: " << msg->alt_ellipsoid << std::endl;
			std::cout << "eph: " << msg->eph << std::endl;
			std::cout << "epv: " << msg->epv << std::endl;
			std::cout << "terrain_alt: " << msg->terrain_alt << std::endl;
			*/
			
			recent_gps_msg = std::make_unique<px4_msgs::msg::VehicleGlobalPosition>(*msg);
			//std::cout << "recent_msg timestamp (should be same as above): " << recent_msg->timestamp << std::endl;
		});
		
		std::cout << "Setup local position subscriber (odometry)..." << std::endl;
		subscription_ned = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			name_prefix + "fmu/out/vehicle_odometry", qos,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
			/*
			std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE LOCAL POSITION DATA"   << std::endl;
			std::cout << "=================================="   << std::endl;
			std::cout << "ts: "      << msg->timestamp    << std::endl;
			std::cout << "x: " << msg->x  << std::endl;
			std::cout << "y: " << msg->y << std::endl;
			std::cout << "z: " << msg->z  << std::endl;
			*/
			
			recent_ned_msg = std::make_unique<px4_msgs::msg::VehicleOdometry>(*msg);
			//std::cout << "recent_ned_msg timestamp (should be same as above): " << recent_ned_msg->timestamp << std::endl;
			
		});
		
		std::cout << "Setup home position..." << std::endl;
		subscription_home = this->create_subscription<px4_msgs::msg::HomePosition>(
			name_prefix + "fmu/out/home_position", qos,
			[this](const px4_msgs::msg::HomePosition::UniquePtr msg) {
			/*
			float64 lat				# Latitude in degrees
			float64 lon				# Longitude in degrees
			float32 alt				# Altitude in meters (AMSL)

			float32 x				# X coordinate in meters
			float32 y				# Y coordinate in meters
			float32 z				# Z coordinate in meters
			*/
			
			recent_home_msg = std::make_unique<px4_msgs::msg::HomePosition>(*msg);
			
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_gps;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_ned;
	rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr subscription_home;
};

/*
int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_global_position listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleGlobalPositionListener>());

	rclcpp::shutdown();
	return 0;
}
*/
