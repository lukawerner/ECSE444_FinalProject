/*
 * motor.c
 *
 *  Created on: Nov 10, 2025
 *      Author: tahas
 */

#include "motor.h"
#include "main.h"

const uint32_t seq_bit = 0b10011000110001000110001000110001;

void stepMotor(int step) {
	uint32_t step_bit = (seq_bit >> step * 4) & 0b1111;
	uint8_t pin = 0b0001;
	for (uint32_t i = 0; i < 4; i++) {
		uint32_t pin_state = (step_bit >> i) & 0b1;
		HAL_GPIO_WritePin(GPIOA, pin, pin_state);
		pin = pin << 1;
	}
}

void stepForward(int steps) {
  for (int i = 0; i < steps; i++) {
    stepMotor(i % 8);
    HAL_Delay(2);
  }
}

void stepBackward(int steps) {
  for (int i = 0; i < steps; i++) {
    stepMotor(7 - (i % 8));
    HAL_Delay(2);
  }
}

int abs(int x) {
	if (x > 0) return x;
	else return -x;
}

void stepDeg(int deg) {
	const float stepsPerDeg = 5.6611f;	// calculated based on the specific motor used (11.375)
	int steps = abs(deg) * stepsPerDeg;
	if (deg > 0) {
		stepForward(steps);
	} else {
		stepBackward(-steps);
	}
}

