/*
 * Copyright (C) 2024-2025 saybur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "FreeRTOS.h"
#include "task.h"

#include "computer.h"
#include "config.h"
#include "control.h"
#include "debug.h"

#include "drivers/serial.h"

#define WATCHDOG_SCRATCH_REG   0

#define CONTROL_WDRST_DEBUG    0xA5A5A5A5

static volatile control_mode_type mode = CONTROL_MODE_IDLE;

static void control_reboot(bool debug)
{
	// set flag to pause restarting, if needed
	if (debug) {
		watchdog_hw->scratch[WATCHDOG_SCRATCH_REG] = CONTROL_WDRST_DEBUG;
	}

	// perform watchdog reset
	watchdog_enable(1, 1);
	while(1);
}

static void control_enqueue(unsigned char c)
{
	if (c >= 0xF0) {
		switch (c) {
		case CONTROL_REBOOT:
			control_reboot(false);
			break;
		case CONTROL_REBOOT_DEBUG:
			control_reboot(true);
			break;
		case CONTROL_START_CONFIG_WRITE:
			mode = CONTROL_MODE_CONFIG_WRITE;
		}
	} else if (mode == CONTROL_MODE_FLYBYWIRE) {
		serial_enqueue(c);
	} else if (mode == CONTROL_MODE_CONFIG_WRITE) {
		bool resp = false;
		config_write_serial_byte(c, &resp);
		if (resp) {
			mode == CONTROL_MODE_IDLE;
		}
	}
}

control_reset_type control_check_reset(void)
{
	// retrieve and clear any special flags
	uint32_t w = watchdog_hw->scratch[WATCHDOG_SCRATCH_REG];
	watchdog_hw->scratch[WATCHDOG_SCRATCH_REG] = 0;

	// if the watchdog was responsible for the reset,
	// check if it was a special condition we need to report
	if (watchdog_enable_caused_reboot()) {
		switch (w) {
			case CONTROL_WDRST_DEBUG:
				return RESET_TYPE_DEBUG;
		}
	}

	// fallback to normal
	return RESET_TYPE_NORMAL;
}

void control_start(void)
{
	mode = CONTROL_MODE_FLYBYWIRE;
}

void control_task(__unused void *parameters)
{
	unsigned char c;
	while (true) {
		c = getc(stdin);
		control_enqueue(c);
	}
}
