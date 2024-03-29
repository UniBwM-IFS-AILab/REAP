/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
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
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Lucas Mair	<lucas.mair@unibw.de>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */
 
// location of this file is at ~/offboard_control_ws/src/px4_ros_com/src/examples/offboard/offboard_control.cpp
 
// including aerostack2 msgs from other workspace (works after calling the relevant <workspace/install>/setup.bash)
#include "as2_msgs/action/takeoff.hpp"
#include "as2_msgs/action/land.hpp"
#include "as2_msgs/srv/get_origin.hpp"
#include "as2_msgs/srv/set_origin.hpp"

// following import is for communication with Plansys2 and based on action model
// see ROS2 tutorial under https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// following include is for transforming gps to NED coordinates
#include "GeodeticConverter.hpp"

// Nodes for listening to px4 messages
// separate header files would be better style, but it should work just fine with including cpp files
#include "vehicle_global_position_listener_lib.cpp"
#include "vehicle_status_listener_lib.cpp"


#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <thread>
#include <iostream>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

//aerostack2 aliases
using TakeOff = as2_msgs::action::Takeoff;
using Land = as2_msgs::action::Land;

using PoseStamped = geometry_msgs::msg::PoseStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;


// utility functions
// Declare str2int() before it is used
constexpr unsigned int str2int(const char* str, int h = 0){
	return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

class OffboardControl : public rclcpp::Node {
public:
	
	std::shared_ptr<VehicleStatusListener> vehicle_status_listener_;
	std::shared_ptr<VehicleGlobalPositionListener> gps_listener_;

	// name_prefix should have the format "<identifier>/"
	OffboardControl(std::shared_ptr<VehicleStatusListener> vehicle_status_listener, std::shared_ptr<VehicleGlobalPositionListener> gps_listener, std::string name_prefix = "") : Node(name_prefix.substr(0, name_prefix.size() - 1)+ "_" + "offboard_control") {
		
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(name_prefix + "fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(name_prefix + "fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(name_prefix + "fmu/in/vehicle_command", 10);

		name_prefix_ = name_prefix;
		
		// get content from listeners
		vehicle_status_listener_ = vehicle_status_listener;
		gps_listener_ = gps_listener;
		
		// set home gps origin for reference
		// possible to get reference frame origin from https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition.html
		// possible to set NED origin via VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN = 100000, normally set on EKF2-module start
		// for simplicity we use the gps_listener->recent_gps_msg before the first arming
		GPS_converter_ = std::make_shared<GeodeticConverter>(gps_listener_->recent_gps_msg->lat, gps_listener_->recent_gps_msg->lon, gps_listener_->recent_gps_msg->alt);
		RCLCPP_INFO(this->get_logger(), "Set GPS Origin to configured PX4 origin (%lf, %lf, %lf)", gps_listener_->recent_gps_msg->lat, gps_listener_->recent_gps_msg->lon, gps_listener_->recent_gps_msg->alt);
		
		auto timer_callback = [this]() -> void {

			// https://docs.px4.io/main/en/flight_modes/offboard.html
			publish_offboard_control_mode();
			
		};
		timer_ = this->create_wall_timer(200ms, timer_callback);
	}
	
	void start_action_server(){
		using namespace std::placeholders;
		
		RCLCPP_INFO(get_logger(), "Starting action server:");

		waypoint_action_server_ = rclcpp_action::create_server<NavigateToPose>(
			shared_from_this(),
			name_prefix_ + "navigate_to_pose",
			std::bind(&OffboardControl::handle_goal<NavigateToPose>, this, _1, _2),
			std::bind(&OffboardControl::handle_cancel<NavigateToPose>, this, _1),
			std::bind(&OffboardControl::handle_accepted_waypoint, this, _1));
			
		takeoff_action_server_ = rclcpp_action::create_server<TakeOff>(
			shared_from_this(),
			name_prefix_ + "takeoff",
			std::bind(&OffboardControl::handle_goal<TakeOff>, this, _1, _2),
			std::bind(&OffboardControl::handle_cancel<TakeOff>, this, _1),
			std::bind(&OffboardControl::handle_accepted_takeoff, this, _1));
			
		landing_action_server_ = rclcpp_action::create_server<Land>(
			shared_from_this(),
			name_prefix_ + "landing",
			std::bind(&OffboardControl::handle_goal<Land>, this, _1, _2),
			std::bind(&OffboardControl::handle_cancel<Land>, this, _1),
			std::bind(&OffboardControl::handle_accepted_landing, this, _1));
		
		sequence_action_server_ = rclcpp_action::create_server<NavigateThroughPoses>(
			shared_from_this(),
			name_prefix_ + "action_sequence",
			std::bind(&OffboardControl::handle_goal<NavigateThroughPoses>, this, _1, _2),
			std::bind(&OffboardControl::handle_cancel<NavigateThroughPoses>, this, _1),
			std::bind(&OffboardControl::handle_accepted_sequence, this, _1));

			RCLCPP_INFO(get_logger(), "Action servers are ready. \n");
	}
	
	void start_services(){
		using namespace std::placeholders;
		
		RCLCPP_INFO(get_logger(), "Starting services:");

		get_origin_service_ = this -> create_service<as2_msgs::srv::GetOrigin>(name_prefix_ + "srv/get_origin", std::bind(&OffboardControl::get_origin, this, _1, _2));
		set_origin_service_ = this -> create_service<as2_msgs::srv::SetOrigin>(name_prefix_ + "srv/set_origin", std::bind(&OffboardControl::set_origin, this, _1, _2));
		

		RCLCPP_INFO(get_logger(), "Services are ready. \n");
	}

	void arm();
	void disarm();

private:

	enum RETURN_VALUE { action_failure = -1, action_completed = 0, goal_succeeded = 4, goal_canceled = 5};
	
	// pointer to action server objects
	rclcpp_action::Server<NavigateToPose>::SharedPtr waypoint_action_server_;
	rclcpp_action::Server<TakeOff>::SharedPtr takeoff_action_server_;
	rclcpp_action::Server<Land>::SharedPtr landing_action_server_;
	rclcpp_action::Server<NavigateThroughPoses>::SharedPtr sequence_action_server_;
	
	// pointer to services
	rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_service_;
	rclcpp::Service<as2_msgs::srv::SetOrigin>::SharedPtr set_origin_service_;

	rclcpp::TimerBase::SharedPtr timer_;
	
	std::shared_ptr<GeodeticConverter> GPS_converter_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	bool is_flying_ = false; //!< boolean for checking if the drone needs to arm and takeoff
	std::string name_prefix_;
	
	void set_home_px4(double latitude = 0.0, double longitude = 0.0, double altitude = 0.0);
	
	void takeoff();
	void land(double latitude = 0.0, double longitude = 0.0, double altitude = 0.0);
	double move_to_gps(double latitude, double longitude, double altitude);
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void hover_in_position();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

	
	// the following functions were derived from ~\companion\PlanSys\src\plansys2_bt_example\src\nav2_sim_node.cpp
	template<typename ActionT>
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename ActionT::Goal> goal){
		std::cout << "In handle_goal \n";
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	
	template<typename ActionT>
	rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle){
		std::cout << "In handle_cancel \n";
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted_waypoint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateToPose>> goal_handle){
		using namespace std::placeholders;
		std::cout << "In handle_accepted_waypoint \n";
		std::thread{std::bind(&OffboardControl::execute_waypoint<NavigateToPose>, this, _1, goal_handle->get_goal()->pose, true), goal_handle}.detach();
	}
	
	void handle_accepted_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TakeOff>> goal_handle){
		using namespace std::placeholders;
		std::cout << "In handle_accepted_takeoff \n";
		std::thread{std::bind(&OffboardControl::execute_takeoff<TakeOff>, this, _1, true), goal_handle}.detach();
	}
	
	void handle_accepted_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> goal_handle){
		using namespace std::placeholders;
		std::cout << "In handle_accepted_landing \n";
		std::thread{std::bind(&OffboardControl::execute_landing<Land>, this, _1, true), goal_handle}.detach();
	}
	
	void handle_accepted_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThroughPoses>> goal_handle){
		using namespace std::placeholders;
		std::cout << "In handle_accepted_sequence \n";
		std::thread{std::bind(&OffboardControl::execute_sequence, this, _1), goal_handle}.detach();
	}
	
	void execute_sequence(const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateThroughPoses>> goal_handle){
		//depending on action call execute waypoint, execute_takeoff or execute_landing
		//check if goal_handle during called subfunction was canceled
		
		auto poses = goal_handle->get_goal()->poses;
		std::string sequence = goal_handle->get_goal()->behavior_tree;
		
		auto result = std::make_shared<NavigateThroughPoses::Result>();
		
		//its possible to abort the goal instead of succeed if it is somehow not possible to execute it
		//goal_handle->abort(result);
		
		//split comma separated behavior_tree string into string vector
		std::istringstream oss(sequence);
		std::string word;
		std::vector<std::string> vtr;
		while(getline(oss, word, ',')) {
			vtr.push_back(word);
		}
		
		int current_pose_index = 0;
		int current_action_index = 0;
		int return_code = -1;
		
		for ( auto it = vtr.begin(); it != vtr.end(); ++it) {
			bool final_action = std::next(it) == vtr.end();
			
			std::cout << "current action in sequence["<< current_action_index <<"]: " << *it << std::endl;
			
			
			
			//use lambda functions to have inline string literals as constexpr to use for str2int
			switch (str2int(it->c_str())) {
				case []{ return str2int("take_off"); }():
					return_code = execute_takeoff<NavigateThroughPoses>(goal_handle);
					break;
				case []{ return str2int("land"); }():
					return_code = execute_landing<NavigateThroughPoses>(goal_handle);
					break;
				case []{ return str2int("fly"); }():
					return_code = execute_waypoint<NavigateThroughPoses>(goal_handle, poses[current_pose_index]);
					current_pose_index++;
					break;
			}
			
			switch (return_code) {
				case OffboardControl::RETURN_VALUE::action_completed:
					//single action in sequence completed
					current_action_index++;
					RCLCPP_INFO(this->get_logger(), "Action %d out of %ld in sequence successfully completed", current_action_index, vtr.size());
					if(final_action){
						RCLCPP_INFO(this->get_logger(), "Sequence successfully completed");
						goal_handle->succeed(result);
					}
					break;
				case OffboardControl::RETURN_VALUE::goal_succeeded:
					//entire sequence succeeded handling
					//instead as condition within OffboardControl::RETURN_VALUE::action_completed
					break;
				case OffboardControl::RETURN_VALUE::goal_canceled:
					//sequence canceled handling
					RCLCPP_INFO(this->get_logger(), "Sequence cancelled during action %d out of %ld",(current_action_index+1), vtr.size());
					RCLCPP_INFO(this->get_logger(), "Successfully completed %d actions of the sequence",(current_action_index+1));
					//TODO: put value into canceled result with number of successfully completed actions/index of canceled action
					//put index in result error code
					goal_handle->canceled(result);
					return;
			}
		}
		return;
	}

	// T should be of type rclcpp_action::ServerGoalHandle<ActionT>
	template<typename ActionT>
	int execute_waypoint(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle, PoseStamped pose, bool final_action = false){
		std::cout << "In execute_waypoint \n";

		//auto pose_cmd = goal_handle->get_goal()->pose.pose;
		auto pose_cmd = pose.pose;

		// RCLCPP_INFO(this->get_logger(), "Starting navigation to %lf, %lf, %lf", pose_cmd.position.x, pose_cmd.position.y, pose_cmd.position.z);
		
		rclcpp::Rate loop_rate(1);
		
		// auto feedback = std::make_shared<typename ActionT::Feedback>();
		// goal_handle->publish_feedback(feedback);
		auto result = std::make_shared<typename ActionT::Result>();
		
		publish_offboard_control_mode();
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
		
		double distance = 0;
		
		int current_times = 0;
		while (rclcpp::ok()) {
			loop_rate.sleep();
			current_times++;
			RCLCPP_INFO(this->get_logger(), "publishing setpoint number %d ", current_times);
			
			// NED frame doesnt reset once reaching a waypoint
			distance = move_to_gps(pose_cmd.position.x, pose_cmd.position.y, pose_cmd.position.z);
			
			// print remaining distance
			RCLCPP_INFO(this->get_logger(), "current distance: %lf ", distance);
			
			// set result to canceled; if part of an action sequence, check separately in calling funtion if canceled from within here
			if (goal_handle->is_canceling()) {
				this->hover_in_position();
				if(final_action){
					goal_handle->canceled(result);
				}
				RCLCPP_INFO(this->get_logger(), "Navigation to current waypoint canceled");
				return OffboardControl::RETURN_VALUE::goal_canceled;
			}
			
			// normally distance <= 5
			if (rclcpp::ok() && distance <= 5) {
				loop_rate.sleep();
				RCLCPP_INFO(this->get_logger(), "Navigation to current waypoint succeeded");
				if(final_action){
					RCLCPP_INFO(this->get_logger(), "in final action");
					goal_handle->succeed(result);
					return OffboardControl::RETURN_VALUE::goal_succeeded;
				}
				break;
			}
		}
		// check in calling function if successfull
		return OffboardControl::RETURN_VALUE::action_completed;
	}
	
	// T should be of type rclcpp_action::ServerGoalHandle<ActionT>
	template<typename ActionT>
	int execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle, bool final_action = false){
		std::cout << "In execute_takeoff \n";
		
		rclcpp::Rate loop_rate(1);
		
		//if constexpr (std::is_same<ActionT, TakeOff>::value) {
		//	float takeoff_height = goal_handle->get_goal()->takeoff_height;
		//}
		
		// auto feedback = std::make_shared<typename ActionT::Feedback>();
		// feedback->actual_takeoff_height = 10;
		// goal_handle->publish_feedback(feedback);
		
		auto result = std::make_shared<typename ActionT::Result>();
		if constexpr (std::is_same<ActionT, TakeOff>::value) {
			result->takeoff_success = false;
		}
		
		publish_offboard_control_mode();
		if(!is_flying_){
			//https://discuss.px4.io/t/where-to-find-custom-mode-list-for-mav-cmd-do-set-mode/32756/4
			
			int retry_count = 0;
			for( ; retry_count < 3; ++retry_count){
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
				
				if (goal_handle->is_canceling()) {
					this->disarm();
					if(final_action){
						goal_handle->canceled(result);
					}
					RCLCPP_INFO(this->get_logger(), "Takeoff canceled");
					return OffboardControl::RETURN_VALUE::goal_canceled;
				}
				
				loop_rate.sleep();
				if (vehicle_status_listener_->recent_status_msg->arming_state == 2) break;
			}
			if (retry_count >= 3){
				RCLCPP_INFO(this->get_logger(), "Takeoff aborted: [could not arm]");
				goal_handle->abort(result);
			}
			
			this->takeoff();
		}
		
		while (rclcpp::ok()) {
			loop_rate.sleep();
			
			// TODO: altitude should be relative to ground...
			// check vehicle state instead of alt > 5 ... (lookup which px4 state represents takeoff vs normal-hover mode)
			if (rclcpp::ok() && gps_listener_->recent_gps_msg->alt > 5) {
				loop_rate.sleep();
				if(final_action){
					if constexpr (std::is_same<ActionT, TakeOff>::value) {
						result->takeoff_success = true;
					}
					goal_handle->succeed(result);
					return OffboardControl::RETURN_VALUE::goal_succeeded;
				}
				RCLCPP_INFO(this->get_logger(), "Takeoff succeeded");
				break;
			}
		}
		return OffboardControl::RETURN_VALUE::action_completed;
	}
	
	template<typename ActionT>
	int execute_landing(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle, bool final_action = false){
		std::cout << "In execute_landing \n";
		
		rclcpp::Rate loop_rate(1);
		
		// auto feedback = std::make_shared<typename ActionT::Feedback>();
		// goal_handle->publish_feedback(feedback);
		
		auto result = std::make_shared<typename ActionT::Result>();
		if constexpr (std::is_same<ActionT, Land>::value) {
			result->land_success = false;
		}
		
		publish_offboard_control_mode();
		if(is_flying_){
			this->land();
			// this->disarm(); <-- not neccessary after land, it disarms automatically
		}
		while (rclcpp::ok()) {
			loop_rate.sleep();
			// TODO: altitude should be relative to ground...
			// check vehicle state landed==disarmed istead of alt < 5 -> https://docs.px4.io/main/en/msg_docs/VehicleStatus.html
			if (rclcpp::ok() && gps_listener_->recent_gps_msg->alt < 5) {
				loop_rate.sleep();
				if(final_action){
					if constexpr (std::is_same<ActionT, Land>::value) {
						result->land_success = true;
					}
					goal_handle->succeed(result);
					return OffboardControl::RETURN_VALUE::goal_succeeded;
				}
				RCLCPP_INFO(this->get_logger(), "Landing succeeded");
				break;
			}
		}
		return OffboardControl::RETURN_VALUE::action_completed;
	}
	
	void get_origin(const std::shared_ptr<as2_msgs::srv::GetOrigin::Request> request, std::shared_ptr<as2_msgs::srv::GetOrigin::Response> response){

		 response->origin.latitude = gps_listener_->recent_home_msg->lat;
		 response->origin.longitude = gps_listener_->recent_home_msg->lon;
		 response->origin.altitude = gps_listener_->recent_home_msg->alt;
		 
		 response->success = true;
	}
	
	
	/**
	 * @brief callback for set_origin service --> actually just setting home. NED origin should stay the same
	 */
	void set_origin(const std::shared_ptr<as2_msgs::srv::SetOrigin::Request> request, std::shared_ptr<as2_msgs::srv::SetOrigin::Response> response){
		
		double lat = request->origin.latitude;
		double lon = request->origin.longitude;
		double alt = request->origin.altitude;
		
		//send new home to px4
		this->set_home_px4(lat, lon, alt);
		
		//wait
		rclcpp::Rate sleep_timer(2);
		sleep_timer.sleep();
		
		//check if home is new one
		response->success = gps_listener_->recent_home_msg->lat == lat && gps_listener_->recent_home_msg->lon == lon && gps_listener_->recent_home_msg->alt == alt;
		
		// set internal origin to request coordinates and then return
		// NOTE!!! should not reset GPS_converter_ because for PX4, home and NED origin are different
		//if (response->success){
		//	GPS_converter_ = std::make_shared<GeodeticConverter>(lat, lon, alt);
		//}
	}
};

	/**
	 * @brief Send a command to set the home position of the vehicle
	 */
	void OffboardControl::set_home_px4(double latitude, double longitude, double altitude) {
		// |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_HOME, 0, 0, 0, 0, latitude, longitude, altitude);
		RCLCPP_INFO(this->get_logger(), "SetHome command send");
	}
	
	
	/**
	 * @brief Send a command to the vehicle to take off 10 meters <-- altitude is not relative to the ground
	 */
	void OffboardControl::takeoff() {
		// Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, gps_listener_->recent_gps_msg->lat, gps_listener_->recent_gps_msg->lon, 10);
		RCLCPP_INFO(this->get_logger(), "Takeoff command send");
		rclcpp::Rate sleep_timer(2);
		sleep_timer.sleep();
		is_flying_ = true;
	}
	
	/**
	 * @brief Send a command to the vehicle to land
	 *        the documentation says its at the specified position (GPS) but in the airsim simulation it just lands at the current position
	 *        see https://discuss.px4.io/t/land-mode-always-land-to-x-0-y-0/33742
	 */
	void OffboardControl::land(double latitude, double longitude, double altitude) {
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0, 0, 0, latitude, longitude, altitude);
		//RCLCPP_INFO(this->get_logger(), "Landing command sent for GPS location: lat %lf, lon %lf, alt %lf", latitude, longitude, altitude);
		RCLCPP_INFO(this->get_logger(), "Landing command sent");
		rclcpp::Rate sleep_timer(2);
		sleep_timer.sleep();
		is_flying_ = false;
	}
	
	/**
	 * @brief Send a command to Arm the vehicle
	 */
	void OffboardControl::arm() {
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

		RCLCPP_INFO(this->get_logger(), "Arm command send");
	}

	/**
	 * @brief Send a command to Disarm the vehicle
	 */
	void OffboardControl::disarm() {
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

		RCLCPP_INFO(this->get_logger(), "Disarm command send");
	}

	/**
	 * @brief Publish the offboard control mode.
	 *        For this example, only position and altitude controls are active.
	 */
	void OffboardControl::publish_offboard_control_mode() {
		OffboardControlMode msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;

		offboard_control_mode_publisher_->publish(msg);
	}


	/**
	 * @brief Publish a trajectory setpoint
	 *        For this example, it sends a trajectory setpoint to make the
	 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
	 */
	void OffboardControl::publish_trajectory_setpoint() {
		TrajectorySetpoint msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		msg.position = {0.0, 0.0, -5.0};
		msg.yaw = -3.14; // [-PI:PI]

		trajectory_setpoint_publisher_->publish(msg);
	}
	
	/**
	 * @brief Publish a trajectory setpoint
	 *        For this example, it sends a trajectory setpoint to make the
	 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
	 */
	void OffboardControl::hover_in_position() {
		TrajectorySetpoint msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		
		double current_north, current_east, current_down;
		
		current_north = gps_listener_->recent_ned_msg->position[0];
		current_east = gps_listener_->recent_ned_msg->position[1];
		current_down = gps_listener_->recent_ned_msg->position[2];
		
		msg.position = {current_north, current_east, current_down};
		msg.yaw = 0; // [-PI:PI]
		
		publish_offboard_control_mode();
		trajectory_setpoint_publisher_->publish(msg);
	}
	
	/**
	 * @brief Publish a trajectory setpoint to a gps coordinate
	 *        returns the distance left to the target
	 *        while moving towards the target, dont decrease the distances, the target NED trajectory point should stay constant
	 */
	double OffboardControl::move_to_gps(double latitude, double longitude, double altitude){
		// NED Coordinates for target
		double north, east, down;
		GPS_converter_->geodetic2Ned(latitude, longitude, altitude, &north, &east, &down);
		
		// Current NED coordinates relative to origin 
		// changed from calculating via geodetic_utils to listening from PX4
		double current_north, current_east, current_down;
		
		// receive current NED coordinates from PX4
		current_north = gps_listener_->recent_ned_msg->position[0];
		current_east = gps_listener_->recent_ned_msg->position[1];
		current_down = gps_listener_->recent_ned_msg->position[2];
		
		double delta_n = north - current_north, delta_e = east - current_east, delta_d = down - current_down;
		// discretize delta values
		delta_n = GPS_converter_->discretize(delta_n,5);
		delta_e = GPS_converter_->discretize(delta_e,5);
		delta_d = GPS_converter_->discretize(delta_d,5);
										
		RCLCPP_INFO(this->get_logger(), "Current GPS position: %lf, %lf, %lf", gps_listener_->recent_gps_msg->lat, gps_listener_->recent_gps_msg->lon, gps_listener_->recent_gps_msg->alt);
		RCLCPP_INFO(this->get_logger(), "Target GPS position: %lf, %lf, %lf", latitude, longitude, altitude);
		RCLCPP_INFO(this->get_logger(), "Current position in NED frame: %lf, %lf, %lf", current_north, current_east, current_down);
		RCLCPP_INFO(this->get_logger(), "Target position in NED frame: %lf, %lf, %lf", north, east, down);

		
		TrajectorySetpoint msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		// waypoints can have a maximum of 900m distance (according to official px4 documentation)
		// smoothing by discretizing points
		north = GPS_converter_->discretize(north,5);
		east = GPS_converter_->discretize(east,5);
		down = GPS_converter_->discretize(down,5);
		RCLCPP_INFO(this->get_logger(), "Smoothing trajectory point to target NED coordinates: %lf, %lf, %lf", north, east, down);
		msg.position = {north, east, down};
		msg.yaw = 0; // [-PI:PI]
		
		publish_offboard_control_mode();
		trajectory_setpoint_publisher_->publish(msg);
		
		return GPS_converter_->getDistance(delta_n, delta_e, delta_d);
	}

	/**
	 * @brief Publish vehicle commands
	 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
	 * @param param1    Command parameter 1
	 * @param param2    Command parameter 2
	 */
	void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
							  float param2, float param3, float param4, float param5, float param6, float param7) {
		VehicleCommand msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		msg.param1 = param1;
		msg.param2 = param2;
		msg.param3 = param3;
		msg.param4 = param4;
		msg.param5 = param5;
		msg.param6 = param6;
		msg.param7 = param7;
		msg.command = command;
		
		// the target_system field makes problems with mutiple drones... set to 0 to ignore
		// related to mavlink system id
		msg.target_system = 0;
		msg.target_component = 1;
		
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;

		vehicle_command_publisher_->publish(msg);
	}

// for running multiple nodes, see as Example https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html
int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	std::string name_prefix = "";
	
	std::cout << "printing argv[]..." << std::endl;
	for (int i = 0; i < argc; ++i)
	{
		RCLCPP_INFO(rclcpp::get_logger("main"), "Argument %d: %s", i, argv[i]);
	}
	
	//set a name prefix only if it was provided externally via a launchfile or by directly running the node
	//keep in mind that ros2 topic/namespace issues are a likely cause if the components of the framework seem to not communicate with each other
	if(argv[1]!=NULL && strcmp(argv[1],"--ros-args")!=0){
		name_prefix = argv[1];
	}
	
	auto status_listener = std::make_shared<VehicleStatusListener>(name_prefix);
	auto gps_listener = std::make_shared<VehicleGlobalPositionListener>(name_prefix);
	
	//spin listener first for a short time so initial status messages arrive
	//executor.spin_node_some(gps_listener); // from https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Executor.html <-- doesnt work
	//alternative with timeout:
	//rclcpp::spin_until_future_complete(gps_listener, gps_listener -> get_next_gps_future(), std::chrono::seconds(10));
	//without timeout:
	rclcpp::spin_until_future_complete(gps_listener, gps_listener -> get_next_gps_future());
	
	auto controller = std::make_shared<OffboardControl>(status_listener, gps_listener, name_prefix);
	controller -> start_action_server();
	controller -> start_services();
	
	rclcpp::executors::SingleThreadedExecutor executor;

	executor.add_node(status_listener);
	executor.add_node(gps_listener);
	executor.add_node(controller);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
