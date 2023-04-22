/*******************************************************************************
 * Copyright (c) 2018 Integrated Device Technology, Inc.
 * All Rights Reserved.
 *
 * This code is proprietary to IDT, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.idt.com/document/msc/idt-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    raq.h
 * @date    2019-11-26
 * @author  IDT
 * @version 2.0.1 - https://semver.org/
 * @brief   This file contains definitions for the data structure and the RAQ
 *          algorithm function definition.
 * @details This file contains the function definitions for the RAQ algorithm.
 */

#ifndef RAQ_H_
#define RAQ_H_

#include "idt_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup current status
 * Status of the sensor.
 * @{
 */

#define ZMOD4450_OK          (0) /**< Sensor stabilized. */
#define ZMOD4450_STABILIZING (9) /**< Sensor not stabilized. */
/** @} */

/**
 * @brief Parameters to control the RAQ.
 */
typedef struct {
    float alpha; /**< Slope parameter for RAQ. */
    uint32_t stop_delay; /**< Control signal follow-up time. */
    float threshold; /**< Threshold to switch, i.e. 1.3 - corresponds to 30 %
                               rise in concentration. */
    uint32_t tau; /**< Time constant for averaging. */
    uint32_t stabilization_samples; /**< Ignore number of samples for sensor
                                       stabilization. */
} raq_params;

/**
 * @brief Control signal states.
 */
typedef enum {
    OFF = 0,
    ON = 1,
} control_signal_state_t;

/**
 * @brief RAQ results.
 */
typedef struct {
    control_signal_state_t cs_state; /**< Control signal input. */
    float conc_ratio; /**< Concentration ratio. */
} raq_results_t;

/**
 * @brief   Calculates RAQ from r_mox and raq parameters.
 * @param   [in] r_mox MOx resistance.
 * @param   [in] params RAQ parameters.
 * @param   [in,out] results RAQ results.
 * @return  Status of the sensor.
 * @retval  0 Sensor stabilized.
 * @retval  9 Sensor not stabilized.
 */
control_signal_state_t calc_raq(float r_mox, raq_params *params,
                                raq_results_t *results);

#ifdef __cplusplus
}
#endif

#endif /* RAQ_H_ */
