#pragma once

#include <cstdint>

#include "config.hpp"  // Joint, joints
#include "motor.hpp"

/* Non-fundamental functions: the six joints in config.hpp, built on the
 * fundamental functions in motor.hpp. They turn joint values into motor
 * values, then call the fundamental functions, which edit the motor's struct
 * and send from it.
 *
 *  - Config mode (the start-up state): nothing is sent, except what is asked.
 *  - Action mode (sim mode): the motion commands work only here, and only
 *    for a motor whose torque is on. Each command is sent once and the motor
 *    keeps it; the 100 Hz loop only reads feedback, for the running tests.
 *
 * Joint values: direction applied (-1 for mirrored joints), position 0 = where
 * a position joint was when its torque was turned on. Commanded speed is the
 * motor's own (before the gearbox); feedback speed is at the output shaft.
 * Functions return nullptr when done, else why not. */

namespace motor_utils {

/* ── joint <-> motor values ── */

/* Joint command → motor command: pos x direction + offset, vel x direction
 * (no gear ratio), tff x direction. Fills out, or returns why it does not fit. */
const char *to_motor(const Joint_struct &j, const motor::Command_struct &command, motor::Command_struct &out);
/* Motor command → joint command, the reverse of to_motor. */
motor::Command_struct to_joint(const Joint_struct &j, const motor::Command_struct &command);

/* Motor feedback → joint feedback: position (counts) - offset, x direction;
 * velocity x direction / gear ratio, in 0.01 RPM at the output shaft. */
void to_joint_feedback(const Joint_struct &j, int16_t position, int16_t velocity,
                       int32_t &joint_position, int32_t &joint_velocity_x100);

/* Starts the 100 Hz loop task, in config mode; call once, after the UARTs
 * and instruction::init_locks(). The task first turns every motor's torque
 * off and sets it to reply only to PING and READ. */
void start();

/* Action: motion commands and tests are allowed. Config: the tests end and
 * every motor's torque is turned off. */
void set_action(bool on);

/* True in action mode. */
bool in_action();

/* Either mode, every motor. On: each one whose torque is off holds where it
 * is (that pose is its 0); if one does not answer, all are turned off. Off:
 * every motor's torque is turned off and the tests end. */
const char *set_torque_all(bool on);

/* Either mode, one motor. On: it holds where it is (that pose is its 0),
 * unless its torque is on already. Off: its torque is turned off and its test
 * ends. */
const char *set_torque(int bus, uint8_t id, bool on);

/* ── action mode only, for a motor whose torque is on ── */

/* set_gains_and_command for one joint, then logs it for samples cycles (LOG
 * lines). Ends a test running on the other bus. */
const char *test_motor(int bus, uint8_t id, const motor::Gains_struct &gains,
                       const motor::Command_struct &command, uint16_t samples);
/* test_motor for that id on both buses at once, with one start time; neither
 * changes unless the command fits both. */
const char *test_motor_dual(uint8_t id, const motor::Gains_struct &gains,
                            const motor::Command_struct &command, uint16_t samples);

}  // namespace motor_utils
