/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Thrust vectoring attitude mode
 *
 * Specifies the type of attitude setpoint sent to the attitude controller.
 * The parameter can be set to a normal attitude setpoint, where the tilt
 * of the vehicle (roll and pitch) are calculated from the desired thrust
 * vector. This should be the behavior for the non-omnidirectional vehicles,
 * such as quadrotors and other multirotors with coplanar rotors.
 * The "constant zero tilt" mode enforces zero roll and pitch and can only be
 * used for omnidirectional vehicles. The min-tilt mode enforces zero tilt
 * up to a maximum set acceleration (thrust) and tilts the vehicle as small
 * as possible if the thrust vector is larger than the maximum. The "constant
 * tilt" and "constant roll/pitch" modes enforce a given tilt or roll/pitch
 * for the vehicle. The estimation modes estimate the optimal tilt or roll/pitch
 * to counteract with the external force (e.g., wind).
 *
 * @min 0
 * @max 2
 * @value 3 tilted attitude
 * @value 1 constant zero tilt
 * @value 2 fixed attitude
 */
PARAM_DEFINE_INT32(VECT_ATT_MODE, 0);


/**
 * Manual orientation
 *
 * @min 0
 * @max 3
 * @value 0 None
 * @value 1 Forward
 * @value 2 Backward
 * @value 2 CCW
 */
PARAM_DEFINE_INT32(MAN_ATT_DIR, 0);


