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
 * Proportional gain for vertical position error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.f);

/**
 * Proportional gain for horizontal position error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.95f);

/**
 * Proportional gain for vertical velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 2
 * @max 15
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P_ACC, 4.f);

/**
 * Proportional gain for horizontal velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 1.2
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P_ACC, 1.8f);

/**
 * Integral gain for vertical velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m velocity integral
 *
 * @min 0.2
 * @max 3
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I_ACC, 2.f);

/**
 * Integral gain for horizontal velocity error
 *
 * Defined as correction acceleration in m/s^2 per m velocity integral
 * Allows to eliminate steady state errors in disturbances like wind.
 *
 * @min 0
 * @max 60
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I_ACC, 0.4f);

/**
 * Differential gain for vertical velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D_ACC, 0.f);

/**
 * Differential gain for horizontal velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D_ACC, 0.2f);


//For planar controller


/**
 * Proportional gain for horizontal position error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_PXY_P, 0.95f);
/**
 * Integral gain for horizontal position error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_PXY_I, 0.95f);
/**
 * Derivative gain for horizontal position error
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_PXY_D, 0.95f);


/**
 * Proportional gain for horizontal velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s velocity error
 *
 * @min 0.0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_PXY_VEL_P_A, 1.8f);



/**
 * Integral gain for horizontal velocity error
 *
 * Defined as correction acceleration in m/s^2 per m velocity integral
 * Allows to eliminate steady state errors in disturbances like wind.
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_PXY_VEL_I_A, 0.4f);


/**
 * Differential gain for horizontal velocity error
 *
 * Defined as corrective acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.0
 * @max 100
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_PXY_VEL_D_A, 0.2f);
