#include "drone/Drone.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

#include "px4_msgs/msg/vehicle_local_position_setpoint.hpp"
#include "tf2/utils.h"

Drone::Drone() {

	//if (argc != 0 && argv != nullptr) {
	//	rclcpp::init(argc, argv);
	//}
	
	this->exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	this->px4_node_ = std::make_shared<rclcpp::Node>("Drone");
	this->exec_->add_node(px4_node_);
	this->spin_thread_ = std::thread(
		[this]() {
			this->exec_->spin();
		}
	);

	rclcpp::QoS qos_profile(10);
	qos_profile.best_effort();

	std::string vehicle_id_prefix{};

	// Essa parte será útil para múltiplos drones
	//if (this->vehicle_id_ != 0) {
	//	vehicle_id_prefix = "/px4_" + std::to_string(this->vehicle_id_);
	//	this->target_system_ = this->vehicle_id_ + 1;
	//}

	this->vehicle_status_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
		vehicle_id_prefix + "/fmu/out/vehicle_status",
		qos_profile,
		[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
		auto set_arm_disarm_reason = [](uint8_t reason)
		{
			DronePX4::ARM_DISARM_REASON value;
			switch (reason) {
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_TRANSITION_TO_STANDBY:
				value = DronePX4::ARM_DISARM_REASON::TRANSITION_TO_STANDBY;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_STICK:
				value = DronePX4::ARM_DISARM_REASON::RC_STICK;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_RC_SWITCH:
				value = DronePX4::ARM_DISARM_REASON::RC_SWITCH;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_INTERNAL:
				value = DronePX4::ARM_DISARM_REASON::COMMAND_INTERNAL;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_COMMAND_EXTERNAL:
				value = DronePX4::ARM_DISARM_REASON::COMMAND_EXTERNAL;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_MISSION_START:
				value = DronePX4::ARM_DISARM_REASON::MISSION_START;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SAFETY_BUTTON:
				value = DronePX4::ARM_DISARM_REASON::SAFETY_BUTTON;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_LAND:
				value = DronePX4::ARM_DISARM_REASON::AUTO_DISARM_LAND;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT:
				value = DronePX4::ARM_DISARM_REASON::AUTO_DISARM_PREFLIGHT;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_KILL_SWITCH:
				value = DronePX4::ARM_DISARM_REASON::KILL_SWITCH;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_LOCKDOWN:
				value = DronePX4::ARM_DISARM_REASON::LOCKDOWN;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_FAILURE_DETECTOR:
				value = DronePX4::ARM_DISARM_REASON::FAILURE_DETECTOR;
				break;
			case px4_msgs::msg::VehicleStatus::ARM_DISARM_REASON_SHUTDOWN:
				value = DronePX4::ARM_DISARM_REASON::SHUTDOWN;
				break;
			default:
				value = DronePX4::ARM_DISARM_REASON::ARM_DISARM_REASON_NONE;
			}
			return value;
		};

		this->arm_reason_ = set_arm_disarm_reason(msg->latest_arming_reason);
		this->disarm_reason_ = set_arm_disarm_reason(msg->latest_disarming_reason);

		switch (msg->failure_detector_status) {
			case px4_msgs::msg::VehicleStatus::FAILURE_NONE:
			this->failure_ = DronePX4::FAILURE::NONE;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_ROLL:
			this->failure_ = DronePX4::FAILURE::ROLL;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_PITCH:
			this->failure_ = DronePX4::FAILURE::PITCH;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_ALT:
			this->failure_ = DronePX4::FAILURE::ALT;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_ARM_ESC:
			this->failure_ = DronePX4::FAILURE::ARM_ESC;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_BATTERY:
			this->failure_ = DronePX4::FAILURE::BATTERY;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_IMBALANCED_PROP:
			this->failure_ = DronePX4::FAILURE::IMBALANCED_PROP;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_MOTOR:
			this->failure_ = DronePX4::FAILURE::MOTOR;
			break;
			case px4_msgs::msg::VehicleStatus::FAILURE_EXT:
			this->failure_ = DronePX4::FAILURE::EXT;
			break;
			default:
			this->failure_ = DronePX4::FAILURE::NONE;
		}

		switch (msg->arming_state) {
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_INIT:
			this->arming_state_ = DronePX4::ARMING_STATE::INIT;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY:
			this->arming_state_ = DronePX4::ARMING_STATE::STANDBY;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
			this->arming_state_ = DronePX4::ARMING_STATE::ARMED;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY_ERROR:
			this->arming_state_ = DronePX4::ARMING_STATE::STANDBY_ERROR;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_SHUTDOWN:
			this->arming_state_ = DronePX4::ARMING_STATE::SHUTTEDDOWN;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_IN_AIR_RESTORE:
			this->arming_state_ = DronePX4::ARMING_STATE::IN_AIR_RESTORE;
			break;
			case px4_msgs::msg::VehicleStatus::ARMING_STATE_MAX:
			this->arming_state_ = DronePX4::ARMING_STATE::MAX;
			break;
			default:
			this->arming_state_ = DronePX4::ARMING_STATE::MAX;
		}

		switch (msg->nav_state) {
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::MANUAL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::ALTCTL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::POSCTL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_MISSION;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_LOITER;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_RTL;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ACRO:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::ACRO;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_TERMINATION:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::TERMINATION;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::OFFBOARD;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::STAB;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_TAKEOFF;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_LAND;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_FOLLOW_TARGET;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_PRECLAND;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::ORBIT;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::AUTO_VTOL_TAKEOFF;
			break;
			case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_DESCEND:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::DESCEND;
			break;
			default:
			this->flight_mode_ = DronePX4::FLIGHT_MODE::UNKNOWN_MODE;
		}
		}
	);

	this->vehicle_timesync_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::TimesyncStatus>(
		vehicle_id_prefix + "/fmu/out/timesync_status",
		qos_profile,
		[this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr msg) {
		this->timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
			std::chrono::nanoseconds(msg->timestamp));
		});

	this->vehicle_odometry_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
		vehicle_id_prefix + "/fmu/out/vehicle_odometry",
		qos_profile,
		[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
			this->odom_timestamp_ = std::chrono::time_point<std::chrono::high_resolution_clock>(
				std::chrono::nanoseconds(msg->timestamp));
			this->ground_speed_ = std::sqrt(
				std::pow(msg->velocity[0], 2) + std::pow(msg->velocity[1], 2));

			this->current_pos_x_ = msg->position[0];
			this->current_pos_y_ = msg->position[1];
			this->current_pos_z_ = msg->position[2];
			this->current_vel_x_ = msg->velocity[0];
			this->current_vel_y_ = msg->velocity[1];
			this->current_vel_z_ = msg->velocity[2];

			// if the quaternion is valid, extract the euler angles for convenience
			if (msg->q[0] != NAN) {
				double y = 0, p = 0, r = 0;
				// the ordering is different: PX4 does wxyz, TF2/Bullet does xyzw
				tf2::getEulerYPR(
					tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]),
					y, p, r
				);
				this->yaw_ = static_cast<float>(y);
				this->pitch_ = static_cast<float>(p);
				this->roll_ = static_cast<float>(r);
			}
		}
	);

	this->vehicle_airspeed_sub_ = this->px4_node_->create_subscription<px4_msgs::msg::Airspeed>(
		vehicle_id_prefix + "/fmu/out/airspeed",
		qos_profile,
		[this](px4_msgs::msg::Airspeed::ConstSharedPtr msg) {
			this->airspeed_ = msg->true_airspeed_m_s;
		}
	);

	this->vehicle_rates_setpoint_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
		vehicle_id_prefix + "/fmu/in/vehicle_rates_setpoint", qos_profile);

	this->vehicle_command_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::VehicleCommand>(
		vehicle_id_prefix + "/fmu/in/vehicle_command", qos_profile);

	this->vehicle_trajectory_setpoint_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		vehicle_id_prefix + "/fmu/in/trajectory_setpoint", qos_profile);

	this->vehicle_offboard_control_mode_pub_ = this->px4_node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
		vehicle_id_prefix + "/fmu/in/offboard_control_mode", qos_profile);

}

DronePX4::ARMING_STATE Drone::getArmingState() {
	return this->arming_state_;
}

DronePX4::FLIGHT_MODE Drone::getFlightMode() {
  	return this->flight_mode_;
}

DronePX4::ARM_DISARM_REASON Drone::getArmReason() {
  	return this->arm_reason_;
}

DronePX4::ARM_DISARM_REASON Drone::getDisarmReason() {
	return this->disarm_reason_;
}

DronePX4::FAILURE Drone::getFailure() {
	return this->failure_;
}


Eigen::Vector3d Drone::getLocalPosition() {
	return Eigen::Vector3d({
		this->current_pos_x_,
		this->current_pos_y_,
		this->current_pos_z_
	});
}


float Drone::getAltitude() {
	return this->current_pos_z_;
}


float Drone::getGroundSpeed() {
	return this->ground_speed_;
}

float Drone::getAirSpeed() {
	return this->airspeed_;
}

Eigen::Vector3d Drone::getOrientation() {
	return Eigen::Vector3d({
		this->roll_,
		this->pitch_,
		this->yaw_
	});
}


void Drone::arm() {
    this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f
	);
}
void Drone::armSync() {
	while (getArmingState() != DronePX4::ARMING_STATE::ARMED && rclcpp::ok()) {
    	this->arm();
    	usleep(1e5);  // 100 ms
  	}
}

void Drone::disarm() {
  	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		0.0f
	);
}

void Drone::disarmSync() {
  	while (getArmingState() != DronePX4::ARMING_STATE::STANDBY && rclcpp::ok()) {
		this->disarm();
		usleep(1e5);  // 100 ms
  	}
}

/*
void Drone::takeoff() {
  	this->sendCommand(
   		// https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		0.1f,  // Minimum pitch (if airspeed sensor present), desired pitch without sensor (degrees)
		0,     // Empty
		0,     // Empty
		1.57,  // Yaw angle (degrees)
		this->lat_,  // Latitude
		this->lon_,  // Longitude
		this->alt_ + 5.0f // Altitude (meters)
	);  
}

void Drone::land() {

}
*/

void Drone::setLocalPosition(float x, float y, float z, float yaw) {
	px4_msgs::msg::TrajectorySetpoint msg;

	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	msg.yaw = yaw;

	// non-NaN velocity and acceleration fields are used as feedforward terms.
	// We will just set them all to NaN, to keep this API simple.

	msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
	msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
	msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
	msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

	msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

	this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setLocalPositionSync(
	double x,
	double y,
	double z,
	double yaw,
	double airspeeed,
	double distance_threshold,
	DronePX4::CONTROLLER_TYPE controller_type)
{
	while (rclcpp::ok()) {
		setOffboardControlMode(controller_type);
		setLocalPosition(x, y, z, yaw);
		setAirSpeed(airspeeed);

		auto currentPosition = getLocalPosition();
		auto diff = currentPosition - Eigen::Vector3d({x,y,z});
		const auto distance = diff.norm();

		if (distance < distance_threshold) {
		break;
		}

		usleep(1e5);  // 100 ms
	}
}

void Drone::setLocalVelocity(float vx, float vy, float vz, float yaw_rate) {
	px4_msgs::msg::TrajectorySetpoint msg;

	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position[0] = std::numeric_limits<float>::quiet_NaN();
	msg.position[1] = std::numeric_limits<float>::quiet_NaN();
	msg.position[2] = std::numeric_limits<float>::quiet_NaN();
	msg.yaw = std::numeric_limits<float>::quiet_NaN();

	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	msg.yawspeed = yaw_rate;

	msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
	msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

	this->vehicle_trajectory_setpoint_pub_->publish(msg);
}

void Drone::setGroundSpeed(float speed) {
	this->setSpeed(speed, true);
}

void Drone::setAirSpeed(float speed) {
  	this->setSpeed(speed, false);
}

void Drone::setOffboardControlMode(DronePX4::CONTROLLER_TYPE type) {
	px4_msgs::msg::OffboardControlMode msg;
	msg.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000;

	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.actuator = false;

	if (type == DronePX4::CONTROLLER_TYPE::POSITION) {
		msg.position = true;
	} else if (type == DronePX4::CONTROLLER_TYPE::VELOCITY) {
		msg.velocity = true;
	} else if (type == DronePX4::CONTROLLER_TYPE::BODY_RATES) {
		msg.body_rate = true;
	} else {
		RCLCPP_WARN(this->px4_node_->get_logger(), "No controller is defined");
	}

	this->vehicle_offboard_control_mode_pub_->publish(msg);
}

void Drone::toOffboardSync() {
	for (int i = 0; i < 20; i++) {
		setLocalPosition(
			0.0,
			0.0,
			std::numeric_limits<float>::quiet_NaN(),
			std::numeric_limits<float>::quiet_NaN());
		setOffboardControlMode(DronePX4::CONTROLLER_TYPE::POSITION);
		usleep(1e5);  // 100 ms
	}
	setOffboardMode();
}

void Drone::setOffboardMode() {
	this->sendCommand(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		this->target_system_,
		this->target_component_,
		this->source_system_,
		this->source_component_,
		this->confirmation_,
		this->from_external_,
		1.0f,
		6.0f
	);
}

void Drone::sendCommand(
	uint32_t command, uint8_t target_system, uint8_t target_component, uint8_t source_system,
	uint8_t source_component, uint8_t confirmation, bool from_external,
	float param1, float param2, float param3,
	float param4, float param5, float param6,
	float param7)
{
	px4_msgs::msg::VehicleCommand msg_vehicle_command;
	msg_vehicle_command.timestamp = this->px4_node_->get_clock()->now().nanoseconds() / 1000.0;
	msg_vehicle_command.command = command;

	msg_vehicle_command.param1 = param1;
	msg_vehicle_command.param2 = param2;
	msg_vehicle_command.param3 = param3;
	msg_vehicle_command.param4 = param4;
	msg_vehicle_command.param5 = param5;
	msg_vehicle_command.param6 = param6;
	msg_vehicle_command.param7 = param7;
	msg_vehicle_command.confirmation = confirmation;
	msg_vehicle_command.source_system = source_system;
	msg_vehicle_command.target_system = target_system;
	msg_vehicle_command.target_component = target_component;
	msg_vehicle_command.from_external = from_external;
	msg_vehicle_command.source_component = source_component;

	this->vehicle_command_pub_->publish(msg_vehicle_command);
}

void Drone::setSpeed(float speed, bool is_ground_speed)
{
  float speed_type = is_ground_speed ? 1.0f : 0.0f;  // true = ground speed, false = air speed
  this->sendCommand(
    // https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_CHANGE_SPEED,
    this->target_system_,
    this->target_component_,
    this->source_system_,
    this->source_component_,
    this->confirmation_,
    this->from_external_,
    speed_type,  // Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
    speed,       // Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
    -1.0f);      // Throttle (-1 indicates no change, -2 indicates return to default throttle value)
}