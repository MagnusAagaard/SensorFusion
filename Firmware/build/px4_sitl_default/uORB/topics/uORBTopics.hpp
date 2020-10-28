/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{160};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_virtual_fw = 6,
	actuator_controls_virtual_mc = 7,
	actuator_outputs = 8,
	adc_report = 9,
	airspeed = 10,
	airspeed_validated = 11,
	battery_status = 12,
	camera_capture = 13,
	camera_trigger = 14,
	camera_trigger_secondary = 15,
	cellular_status = 16,
	collision_constraints = 17,
	collision_report = 18,
	commander_state = 19,
	cpuload = 20,
	debug_array = 21,
	debug_key_value = 22,
	debug_value = 23,
	debug_vect = 24,
	differential_pressure = 25,
	distance_sensor = 26,
	ekf2_timestamps = 27,
	ekf_gps_drift = 28,
	ekf_gps_position = 29,
	esc_report = 30,
	esc_status = 31,
	estimator_innovation_test_ratios = 32,
	estimator_innovation_variances = 33,
	estimator_innovations = 34,
	estimator_sensor_bias = 35,
	estimator_status = 36,
	follow_target = 37,
	fw_virtual_attitude_setpoint = 38,
	geofence_result = 39,
	gps_dump = 40,
	gps_inject_data = 41,
	home_position = 42,
	hover_thrust_estimate = 43,
	input_rc = 44,
	iridiumsbd_status = 45,
	irlock_report = 46,
	landing_gear = 47,
	landing_target_innovations = 48,
	landing_target_pose = 49,
	led_control = 50,
	log_message = 51,
	logger_status = 52,
	manual_control_setpoint = 53,
	mavlink_log = 54,
	mc_virtual_attitude_setpoint = 55,
	mission = 56,
	mission_result = 57,
	mount_orientation = 58,
	multirotor_motor_limits = 59,
	obstacle_distance = 60,
	obstacle_distance_fused = 61,
	offboard_control_mode = 62,
	onboard_computer_status = 63,
	optical_flow = 64,
	orb_multitest = 65,
	orb_test = 66,
	orb_test_large = 67,
	orb_test_medium = 68,
	orb_test_medium_multi = 69,
	orb_test_medium_queue = 70,
	orb_test_medium_queue_poll = 71,
	orbit_status = 72,
	parameter_update = 73,
	ping = 74,
	position_controller_landing_status = 75,
	position_controller_status = 76,
	position_setpoint = 77,
	position_setpoint_triplet = 78,
	power_button_state = 79,
	power_monitor = 80,
	pwm_input = 81,
	qshell_req = 82,
	qshell_retval = 83,
	radio_status = 84,
	rate_ctrl_status = 85,
	rc_channels = 86,
	rc_parameter_map = 87,
	rpm = 88,
	safety = 89,
	satellite_info = 90,
	sensor_accel = 91,
	sensor_accel_fifo = 92,
	sensor_accel_integrated = 93,
	sensor_accel_status = 94,
	sensor_baro = 95,
	sensor_combined = 96,
	sensor_correction = 97,
	sensor_gyro = 98,
	sensor_gyro_fifo = 99,
	sensor_gyro_integrated = 100,
	sensor_gyro_status = 101,
	sensor_mag = 102,
	sensor_preflight = 103,
	sensor_selection = 104,
	servorail_status = 105,
	subsystem_info = 106,
	system_power = 107,
	task_stack_info = 108,
	tecs_status = 109,
	telemetry_status = 110,
	test_motor = 111,
	timesync = 112,
	timesync_status = 113,
	trajectory_bezier = 114,
	trajectory_setpoint = 115,
	trajectory_waypoint = 116,
	transponder_report = 117,
	tune_control = 118,
	uavcan_parameter_request = 119,
	uavcan_parameter_value = 120,
	ulog_stream = 121,
	ulog_stream_ack = 122,
	vehicle_acceleration = 123,
	vehicle_air_data = 124,
	vehicle_angular_acceleration = 125,
	vehicle_angular_velocity = 126,
	vehicle_angular_velocity_groundtruth = 127,
	vehicle_attitude = 128,
	vehicle_attitude_groundtruth = 129,
	vehicle_attitude_setpoint = 130,
	vehicle_command = 131,
	vehicle_command_ack = 132,
	vehicle_constraints = 133,
	vehicle_control_mode = 134,
	vehicle_global_position = 135,
	vehicle_global_position_groundtruth = 136,
	vehicle_gps_position = 137,
	vehicle_imu = 138,
	vehicle_land_detected = 139,
	vehicle_local_position = 140,
	vehicle_local_position_groundtruth = 141,
	vehicle_local_position_setpoint = 142,
	vehicle_magnetometer = 143,
	vehicle_mocap_odometry = 144,
	vehicle_odometry = 145,
	vehicle_rates_setpoint = 146,
	vehicle_roi = 147,
	vehicle_status = 148,
	vehicle_status_flags = 149,
	vehicle_trajectory_bezier = 150,
	vehicle_trajectory_waypoint = 151,
	vehicle_trajectory_waypoint_desired = 152,
	vehicle_vision_attitude = 153,
	vehicle_visual_odometry = 154,
	vehicle_visual_odometry_aligned = 155,
	vtol_vehicle_status = 156,
	wheel_encoders = 157,
	wind_estimate = 158,
	yaw_estimator_status = 159,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
