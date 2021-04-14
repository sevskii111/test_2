/****************************************************************************
 *
 *   Copyright 2019 NXP.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file nxpcup_main.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include "nxpcup_start.h"

#include "nxpcup_race.h"

using namespace matrix;

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit = false;
bool threadIsRunning = false;

#ifdef REAL_CAR
void roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp)
{
	// Converting steering value from percent to euler angle
	control.steer *= 60.0f; //max turn angle 60 degree

	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	Eulerf euler{0.0, 0.0, control.steer};
	Quatf qe{euler};

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	_att_sp.q_d[0] = qe(0);
	_att_sp.q_d[1] = qe(1);
	_att_sp.q_d[2] = qe(2);
	_att_sp.q_d[3] = qe(3);

}
#else
void roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp, vehicle_attitude_s &att)
{
	// Converting steering value from percent to euler angle
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159 / 180; // change to radians

	// Get current attitude quaternion
	matrix::Quatf current_qe{att.q[0], att.q[1], att.q[2], att.q[3]};

	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	matrix::Eulerf euler{0.0, 0.0, control.steer};
	matrix::Quatf qe{euler};

	// Create new quaternion from the difference of current vs steering
	matrix::Quatf new_qe;
	new_qe = current_qe * qe.inversed();

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	new_qe.copyTo(_att_sp.q_d);

}
#endif

int race_thread_main(int argc, char **argv)
{
	threadIsRunning = true;

#ifndef ROVER_INIT_VALUES
#define ROVER_INIT_Values

	/* Publication of uORB messages */
	uORB::Publication<vehicle_control_mode_s>		_control_mode_pub{ORB_ID(vehicle_control_mode)};
	struct vehicle_control_mode_s				_control_mode {};

	uORB::Publication<vehicle_attitude_setpoint_s>		_att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	struct vehicle_attitude_setpoint_s			_att_sp;

	/* Publication of uORB messages */

	/* Return motor control variables */
	roverControl motorControl;

	/* Pixy2 Instance */
#ifdef REAL_CAR
	Pixy2 pixy;
	bool wait = 1;		// needed for waiting for valid data


#endif
	usleep(5000);		// give pixy time to init
#endif

#ifdef REAL_CAR
	PX4_INFO("Real car!\n");

	if (pixy.init() != 0) { return -1; }

	pixy.getVersion();
	pixy.version->print();
#else
	uORB::SubscriptionData<vehicle_attitude_s> att_sub {ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<pixy_vector_s> pixy_sub{ORB_ID(pixy_vector)};
#endif

	usleep(1000);


	while (1) {
		_control_mode.flag_control_manual_enabled = false;
		_control_mode.flag_control_attitude_enabled = true;
		_control_mode.flag_control_velocity_enabled = false;
		_control_mode.flag_control_position_enabled = false;
#ifdef REAL_CAR
		//PX4_INFO("Real car!\n");
		pixy.line.getAllFeatures(LINE_VECTOR, wait);		// get line vectors from pixy
		PX4_INFO("Test: %d\n", pixy.line.vectors[0].m_x0);

		motorControl = raceTrack(pixy);
		roverSteerSpeed(motorControl, _att_sp);		// setting values for speed and steering to attitude setpoints
#else
		pixy_sub.update();
		const pixy_vector_s &pixy_lines = pixy_sub.get();
		PX4_INFO("Test: %d\n", pixy_lines.m0_x0);


		motorControl = raceTrack();
		att_sub.update();
		struct vehicle_attitude_s att = att_sub.get();
		roverSteerSpeed(motorControl, _att_sp, att);
#endif
		PX4_INFO("Speed: %.2f\tSteer: %.2f\n", (double)motorControl.speed, (double)motorControl.steer);

		// Publishing all
		_control_mode.timestamp = hrt_absolute_time();
		_control_mode_pub.publish(_control_mode);
		_att_sp.timestamp = hrt_absolute_time();
		_att_sp_pub.publish(_att_sp);

		if (threadShouldExit) {
			threadIsRunning = false;
			// reset speed and steering
#ifdef REAL_CAR

			roverSteerSpeed(motorControl, _att_sp);		// setting values for speed and steering to attitude setpoints
#else
			roverSteerSpeed(motorControl, _att_sp, att);
#endif
			// puplishing attitude setpoints
			_att_sp.timestamp = hrt_absolute_time();
			_att_sp_pub.publish(_att_sp);

			// Setting vehicle into the default state
			_control_mode.flag_control_manual_enabled 	= true;
			_control_mode.flag_control_attitude_enabled 	= true;
			_control_mode.flag_control_velocity_enabled 	= true;
			_control_mode.flag_control_position_enabled	= true;
			_control_mode.timestamp = hrt_absolute_time();
			_control_mode_pub.publish(_control_mode);

			PX4_INFO("Exit Rover Thread!\n");
			return 1;
		}

	}

	return 0;
}


extern "C" __EXPORT int nxpcup_main(int argc, char *argv[]);
int nxpcup_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: nxpcup {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit = false;
		daemon_task = px4_task_spawn_cmd("nxpcup",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 race_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: race {start|stop|status}\n");
	return 1;
}
