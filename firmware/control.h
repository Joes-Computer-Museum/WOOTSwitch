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

#ifndef __CONTROL_H__
#define __CONTROL_H__

#define CONTROL_REBOOT_IF_IDLE    0xF0
#define CONTROL_REBOOT_ALWAYS     0xF1
#define CONTROL_REBOOT_DEBUG      0xF2

typedef enum {
	RESET_TYPE_NORMAL = 0,
	RESET_TYPE_DEBUG
} control_reset_type;

/**
 * Indicates if there was a special reset condition that should change the
 * device startup mode.
 *
 * @return any special device startup flag, 0 if none is present.
 */
control_reset_type control_check_reset(void);

/**
 * Indicates to the control system that system startup is complete and it may
 * begin performing non-core functions.
 */
void control_start(void);

/**
 * Task responsible for the serial control interface. Users should not call
 * this function.
 */
void control_task(void *parameters);

#endif /* __CONTROL_H__ */
