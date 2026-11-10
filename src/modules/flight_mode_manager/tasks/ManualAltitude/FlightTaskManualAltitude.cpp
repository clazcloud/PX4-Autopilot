/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file FlightTaskManualAltitude.cpp
 */

#include "FlightTaskManualAltitude.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <geo/geo.h>

using namespace matrix;

bool FlightTaskManualAltitude::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sticks.checkAndUpdateStickInputs();

	if (_sticks_data_required) {
		ret = ret && _sticks.isAvailable();
	}

	// in addition to manual require valid position and velocity in D-direction and valid yaw
	return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
}

bool FlightTaskManualAltitude::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0.f;
	_acceleration_setpoint = Vector3f(0.f, 0.f, NAN); // altitude is controlled from position/velocity
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.f;
	_stick_yaw.reset(_yaw, _unaided_yaw);
	_setDefaultConstraints();

	_updateConstraintsFromEstimator();

	return ret;
}

void FlightTaskManualAltitude::_updateConstraintsFromEstimator()
{
	if (PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_min)) {
		_min_distance_to_ground = _sub_vehicle_local_position.get().hagl_min;

	} else {
		_min_distance_to_ground = -INFINITY;
	}

	if (!PX4_ISFINITE(_max_distance_to_ground) && PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_max_z)) {
		_max_distance_to_ground = _sub_vehicle_local_position.get().hagl_max_z;
	}
}

void FlightTaskManualAltitude::_scaleSticks()
{
	// Use sticks input with deadzone and exponential curve for vertical velocity
	const float vel_max_up = fminf(_param_mpc_z_vel_max_up.get(), _velocity_constraint_up);
	const float vel_max_down = fminf(_param_mpc_z_vel_max_dn.get(), _velocity_constraint_down);
	const float vel_max_z = (_sticks.getThrottleZeroCentered() < 0.f) ? vel_max_down : vel_max_up;
	_velocity_setpoint(2) = vel_max_z * -_sticks.getThrottleZeroCenteredExpo();
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	// Depending on stick inputs and velocity, position is locked.
	// If not locked, altitude setpoint is set to NAN.

	// Check if user wants to break
	const bool apply_brake = fabsf(_sticks.getThrottleZeroCenteredExpo()) <= FLT_EPSILON;

	// Check if vehicle has stopped
	const bool stopped = (_param_mpc_hold_max_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _param_mpc_hold_max_z.get());

	// Manage transition between use of distance to ground and distance to local origin
	// when terrain hold behaviour has been selected.
	if (_param_mpc_alt_mode.get() == 2) {
		// Use horizontal speed as a transition criteria
		float spd_xy = Vector2f(_velocity).length();

		// Use presence of horizontal stick inputs as a transition criteria
		float stick_xy = Vector2f(_sticks.getPitchRollExpo()).length();
		bool stick_input = stick_xy > 0.001f;

		if (_terrain_hold) {
			bool too_fast = spd_xy > _param_mpc_hold_max_xy.get();

			if (stick_input || too_fast || !PX4_ISFINITE(_dist_to_bottom)) {
				// Stop using distance to ground
				_terrain_hold = false;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_dist_to_ground_lock) && PX4_ISFINITE(_dist_to_bottom)) {
					_position_setpoint(2) = _position(2) - (_dist_to_ground_lock - _dist_to_bottom);

				} else {
					_position_setpoint(2) = _position(2);
					_dist_to_ground_lock = NAN;
				}
			}

		} else {
			bool not_moving = spd_xy < 0.5f * _param_mpc_hold_max_xy.get() && stopped;

			if (!stick_input && not_moving && PX4_ISFINITE(_dist_to_bottom)) {
				// Start using distance to ground
				_terrain_hold = true;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_position_setpoint(2))) {
					_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));
				}
			}
		}

	}

	if ((_param_mpc_alt_mode.get() == 1 || _terrain_hold) && PX4_ISFINITE(_dist_to_bottom)) {
		// terrain following
		_terrainFollowing(apply_brake, stopped);

	} else {
		// normal mode where height is dependent on local frame

		if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2))) {
			// lock position
			_position_setpoint(2) = _position(2);

			// Ensure that minimum altitude is respected if
			// there is a distance sensor and distance to bottom is below minimum.
			if (PX4_ISFINITE(_dist_to_bottom) && _dist_to_bottom < _min_distance_to_ground) {
				_terrainFollowing(apply_brake, stopped);

			} else {
				_dist_to_ground_lock = NAN;
			}

		} else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake) {
			// Position is locked but check if a reset event has happened.
			// We will shift the setpoints.
			if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counter) {
				_position_setpoint(2) = _position(2);
				_reset_counter = _sub_vehicle_local_position.get().z_reset_counter;
			}

		} else  {
			// user demands velocity change
			_position_setpoint(2) = NAN;
			// ensure that maximum altitude is respected
		}
	}

	_respectMaxAltitude();
}

void FlightTaskManualAltitude::_respectMinAltitude()
{
	// Height above ground needs to be limited (flow / range-finder)
	if (PX4_ISFINITE(_dist_to_bottom) && (_dist_to_bottom < _min_distance_to_ground)) {
		// increase altitude to minimum flow distance
		_position_setpoint(2) = _position(2) - (_min_distance_to_ground - _dist_to_bottom);
	}
}

void FlightTaskManualAltitude::_terrainFollowing(bool apply_brake, bool stopped)
{
	if (apply_brake && stopped && !PX4_ISFINITE(_dist_to_ground_lock)) {
		// User wants to break and vehicle reached zero velocity. Lock height to ground.

		// lock position
		_position_setpoint(2) = _position(2);
		// ensure that minimum altitude is respected
		_respectMinAltitude();
		// lock distance to ground but adjust first for minimum altitude
		_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));

	} else if (apply_brake && PX4_ISFINITE(_dist_to_ground_lock)) {
		// vehicle needs to follow terrain

		// difference between the current distance to ground and the desired distance to ground
		const float delta_distance_to_ground = _dist_to_ground_lock - _dist_to_bottom;
		// adjust position setpoint for the delta (note: NED frame)
		_position_setpoint(2) = _position(2) - delta_distance_to_ground;

	} else {
		// user demands velocity change in D-direction
		_dist_to_ground_lock = _position_setpoint(2) = NAN;
	}
}

void FlightTaskManualAltitude::_respectMaxAltitude()
{
	if (PX4_ISFINITE(_dist_to_bottom)) {

		float vel_constrained = _param_mpc_z_p.get() * (_max_distance_to_ground - _dist_to_bottom);

		if (PX4_ISFINITE(_max_distance_to_ground)) {
			_constraints.speed_up = math::constrain(vel_constrained, -_param_mpc_z_vel_max_dn.get(), _param_mpc_z_vel_max_up.get());

		} else {
			_constraints.speed_up = _param_mpc_z_vel_max_up.get();
		}

		if (_dist_to_bottom > _max_distance_to_ground && !(_sticks.getThrottleZeroCenteredExpo() < FLT_EPSILON)) {
			_velocity_setpoint(2) = math::constrain(-vel_constrained, 0.f, _param_mpc_z_vel_max_dn.get());
		}

		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}
}

void FlightTaskManualAltitude::_respectGroundSlowdown()
{
	// Interpolate descent rate between the altitudes MPC_LAND_ALT1 and MPC_LAND_ALT2
	if (PX4_ISFINITE(_dist_to_ground)) {
		const float limit_down = math::interpolate(_dist_to_ground,
					 _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					 _param_mpc_land_speed.get(), _constraints.speed_down);
		const float limit_up = math::interpolate(_dist_to_ground,
				       _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
				       _param_mpc_tko_speed.get(), _constraints.speed_up);
		_velocity_setpoint(2) = math::constrain(_velocity_setpoint(2), -limit_up, limit_down);
	}
}

void FlightTaskManualAltitude::_ekfResetHandlerHeading(float delta_psi)
{
	// Only reset the yaw setpoint when the heading is locked
	if (PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint = wrap_pi(_yaw_setpoint + delta_psi);
	}

	_stick_yaw.ekfResetHandler(delta_psi);
}

void FlightTaskManualAltitude::_ekfResetHandlerHagl(float delta_hagl)
{
	_dist_to_ground_lock = NAN;
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	_updateYawSetpoint();
	_updateXYSetpoint();
	_updateAltitudeLock();
	_respectGroundSlowdown();
}

void FlightTaskManualAltitude::_updateYawSetpoint()
{
	if (_gov_state == GovState::CYCLIC || _gov_state == GovState::SPIN_DOWN) {
		// Governor owns yaw: ramp the spin rate and ignore the yaw stick.
		// Counterclockwise seen from above is a negative yaw rate in PX4 (NED frame).
		float target_rate = 0.f;                               // spin-down: toward zero
		float accel = math::radians(_param_gov_acc_dn.get());

		if (_gov_state == GovState::CYCLIC) {
			target_rate = -math::radians(_param_gov_rate.get()); // spin-up: toward -GOV_RATE
			accel = math::radians(_param_gov_acc_up.get());
		}

		const float max_change = accel * _deltatime;
		_gov_rate_sp = math::constrain(target_rate, _gov_rate_sp - max_change, _gov_rate_sp + max_change);

		_yawspeed_setpoint = _gov_rate_sp;
		_yaw_setpoint = NAN; // no heading lock while the governor owns yaw

	} else {
		_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint,
					       _sticks.getYawExpo(), _yaw, _deltatime,
					       _unaided_yaw);
	}
}
void FlightTaskManualAltitude::_updateXYSetpoint()
{
	if (_gov_state != GovState::OFF) {
		// Governor: headless tilt.
		// Sticks steer relative to a fixed compass heading (GOV_REF_HDG), not the nose.
		const float ref_heading = math::radians(_param_gov_ref_hdg.get());

		// Scale sticks so full stick gives GOV_TILT_MAX instead of MPC_MAN_TILT_MAX.
		// (Tilt angle maps to acceleration through tan(angle), hence tanf.)
		const float normal_tilt = math::radians(math::max(_param_mpc_man_tilt_max.get(), 1.f));
		const float gov_tilt = math::radians(_param_gov_tilt_max.get());
		const float tilt_scale = tanf(gov_tilt) / tanf(normal_tilt);

		const Vector2f stick = _sticks.getPitchRoll() * tilt_scale;

		_acceleration_setpoint.xy() = _stick_tilt_xy.generateAccelerationSetpoints(
						      stick, _deltatime, _yaw, ref_heading);

	} else {
		// Normal Altitude mode: sticks are relative to the nose.
		_acceleration_setpoint.xy() = _stick_tilt_xy.generateAccelerationSetpoints(
						      _sticks.getPitchRoll(), _deltatime, _yaw, _yaw_setpoint);
	}
}

bool FlightTaskManualAltitude::_checkTakeoff()
{
	// stick is deflected above 65% throttle (throttle stick is in the range [-1,1])
	return _sticks.getThrottleZeroCentered() > 0.3f;
}

bool FlightTaskManualAltitude::update()
{
	bool ret = FlightTask::update();
	_updateGovernorRequest();
	_updateConstraintsFromEstimator();
	_scaleSticks();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();
	_max_distance_to_ground = INFINITY;

	return ret;
}

void FlightTaskManualAltitude::_updateGovernorRequest()
{
	// --- Input 1: channel 8 switch via AUX1 (-1 low, 0 middle, +1 high) ---
	manual_control_setpoint_s manual_control_setpoint;
	float aux1 = -1.f; // no valid data counts as switch low

	if (_manual_control_setpoint_sub.copy(&manual_control_setpoint) && manual_control_setpoint.valid) {
		aux1 = manual_control_setpoint.aux1;
	}

	// --- Input 2: GOV_EN decides what is being asked for ---
	// 0 = off, 1 = switch, 2 = force headless, 3 = force cyclic
	GovState requested = GovState::OFF;

	switch (_param_gov_en.get()) {
	case 1:
		if (aux1 > 0.5f) {
			requested = GovState::CYCLIC;

		} else if (aux1 > -0.5f) {
			requested = GovState::HEADLESS;
		}

		break;

	case 2:
		requested = GovState::HEADLESS;
		break;

	case 3:
		requested = GovState::CYCLIC;
		break;

	default:
		break;
	}

	// --- Input 3: only allowed in Altitude mode ---
	vehicle_status_s vehicle_status;
	bool altitude_mode = false;

	if (_vehicle_status_sub.copy(&vehicle_status)) {
		altitude_mode = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL);
	}

	const bool blocked = (requested != GovState::OFF) && !altitude_mode;

	if (blocked && !_gov_blocked) {
		PX4_WARN("Governor requested but only allowed in Altitude mode");
	}

	_gov_blocked = blocked;

	if (!altitude_mode) {
		requested = GovState::OFF;
	}

	// --- Input 4: measured yaw rate (body z, close to earth vertical at small tilt) ---
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.copy(&angular_velocity)) {
		_gov_measured_rate = angular_velocity.xyz[2];
	}

	// --- Rules: move at most one step per cycle ---
	GovState new_state = _gov_state;

	switch (_gov_state) {
	case GovState::OFF:
		if (requested != GovState::OFF) {
			new_state = GovState::HEADLESS; // always enter through headless
		}

		break;

	case GovState::HEADLESS:
		if (requested == GovState::OFF) {
			new_state = GovState::OFF;

		} else if (requested == GovState::CYCLIC) {
			new_state = GovState::CYCLIC;
		}

		break;

	case GovState::CYCLIC:
		if (requested != GovState::CYCLIC) {
			new_state = GovState::SPIN_DOWN; // always slow down before leaving the spin
		}

		break;

	case GovState::SPIN_DOWN: {
			const float exit_rate = math::radians(_param_gov_exit_rate.get());
			const bool finished = (fabsf(_gov_rate_sp) < 0.01f) && (fabsf(_gov_measured_rate) < exit_rate);

			if (requested == GovState::CYCLIC) {
				new_state = GovState::CYCLIC; // spin back up from wherever it is

			} else if (finished) {
				new_state = GovState::HEADLESS;
			}
		}
		break;
	}

	// --- Act on a change of state ---
	if (new_state != _gov_state) {
		if (new_state == GovState::CYCLIC || new_state == GovState::SPIN_DOWN) {
			// Start every ramp from the yaw rate the airframe actually has
			_gov_rate_sp = _gov_measured_rate;
		}

		if (_gov_state == GovState::SPIN_DOWN && new_state == GovState::HEADLESS) {
			// Spin finished: restart normal yaw handling from the current heading (no stale target)
			_stick_yaw.reset(_yaw, _unaided_yaw);
		}

		_gov_state = new_state;

		const char *names[] = {"OFF", "HEADLESS", "CYCLIC", "SPIN_DOWN"};
		PX4_INFO("Governor: %s", names[static_cast<int>(new_state)]);
	}
}
