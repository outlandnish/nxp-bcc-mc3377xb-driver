/*
 * Copyright 2016 - 2019 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file bcc.c
 *
 * Battery cell controller SW driver V1.1.
 * Supports boards based on MC33771B and MC33772B.
 *
 * This module is common for all supported models.
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Cell map for 7 cells connected to MC33771. */
#define BCC_CM_MC33771_7CELLS     0x380FU
/*! @brief Cell map for 3 cells connected to MC33772. */
#define BCC_CM_MC33772_3CELLS     0x0023U

/*! @brief Time after VPWR connection for the IC to be ready for initialization
 *  (t_VPWR(READY)) in [ms]. */
#define BCC_T_VPWR_READY_MS       5U

/*! @brief CSB_TX LOW period in CSB_TX wake-up pulse sequence (t_1, typ.) in [us]. */
#define BCC_T_WAKE_T1_US          21U

/*! @brief CSB_TX HIGH period in CSB_TX wake-up pulse sequence (t_2, typ.) in [us]. */
#define BCC_T_WAKE_T2_US          600U

/*! @brief EN LOW to HIGH transition to INTB verification pulse
 * (t_INTB_PULSE_DELAY, maximum) in [us]. */
#define BCC_T_INTB_PULSE_DELAY_US 100U

/*! @brief INTB verification pulse duration (t_INTB_PULSE, typ.) in [us]. */
#define BCC_T_INTB_PULSE_US       100U

/*! @brief RESET de-glitch filter (t_RESETFLT, typ.) in [us]. */
#define BCC_T_RESETFLT_US         100U