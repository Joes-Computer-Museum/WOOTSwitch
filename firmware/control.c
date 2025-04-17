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
#include "control.h"
#include "debug.h"

#include "drivers/serial.h"

static void control_reboot(bool always)
{
	if (!always && computer_is_live()) {
		dbg("comp active, reboot cancelled");
		return;
	}

	// perform watchdog reset
	watchdog_enable(1, 1);
	while(1);
}

static void control_enqueue(unsigned char c)
{
	if (c >= 0xF0) {
		switch (c) {
		case CONTROL_REBOOT_IF_IDLE:
			control_reboot(false);
			break;
		case CONTROL_REBOOT_ALWAYS:
			control_reboot(true);
			break;
		}
	} else {
		serial_enqueue(c);
	}
}

void control_task(__unused void *parameters)
{
	unsigned char c;
	while (true) {
		c = getc(stdin);
		control_enqueue(c);
	}
}
