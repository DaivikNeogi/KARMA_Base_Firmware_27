/*
 * robot.h
 *
 *  Created on: Aug 20, 2026
 *      Author: daivi
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void robot_init(void);
void robot_loop(void);
void robot_process_rx(uint8_t *buf, uint32_t len);

#endif /* ROBOT_H */
