#pragma once

#include "pico/types.h"

#define PULSES_PER_REV 0.5f

typedef struct {
	int FAULT_pin_;
	int SLEEP_pin_;
	int DRVOFF_pin_;
	int SCS_pin_;
	int	BRAKE_pin_;
	int PWM_pin_;
	int FGOUT_pin_;
	int pwm_slice_;
	int set_speed_;
} motor_t;

typedef struct {
    int enabled_;
    int speed_;
    int last_pulse_time_;
    float rpm_;
} motor_state_t;

motor_t motor_init(int FAULT_pin, 
					int SLEEP_pin, 
					int DRVOFF_pin,
					int SCS_pin,
					int BRAKE_pin,
					int PWM_pin,
					int FGOUT_pin);

void motor_enable(motor_t* motor);
void motor_disable(motor_t* motor);
void motor_set_speed(motor_t* motor, uint16_t speed);
void motor_reset_fault(motor_t* motor);
uint8_t motor_read_register(motor_t* motor, uint8_t reg_addr);
uint8_t motor_write_register(motor_t* motor, uint8_t reg_addr, uint8_t data);

void fgout_irq(uint gpio, uint32_t events);
