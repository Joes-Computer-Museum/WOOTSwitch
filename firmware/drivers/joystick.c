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

#include <string.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "handler.h"
#include "hardware.h"
#include "host.h"
#include "host_err.h"
#include "host_sync.h"

#include "joystick.h"

/*
 * Driver for ADB joysticks following the Gravis protocol. There are two
 * handlers supported: 0x4E (Firebird/Blackhawk) and 0x23 (Mousestick). These
 * are described in good detail here:
 *
 * <https://github.com/lampmerchant/tashnotes/tree/main/macintosh/adb/protocols>
 *
 * The following assumes familiarity with the above description.
 *
 * The Firebird/Blackhawk is straightforward, this driver just echoes the
 * same information given by the joystick position data.
 *
 * The Mousestick always reports as a 0x0400 (3-byte protocol) device and only
 * supports that type of physical joystick. This is hopefully temporary and I
 * plan to support 0x0300 in the future.
 *
 * TODO this needs cross-joystick handling added (Firebird <-> Mousestick
 * conversion).
 */

#define DEFAULT_ADDRESS 3
#define DEFAULT_HANDLER 1
#define MAX_DEVICES 2

typedef enum {
	MODE_INVALID = 0,
	MODE_FIREBIRD,
	MODE_MOUSESTICK
} device_mode;

typedef struct {
	uint8_t hdev;
	uint8_t hdhi;
	uint8_t drv_idx;
	uint8_t dhi[COMPUTER_COUNT];
	device_mode mode;
	uint8_t reg1[3];    // register 1 from physical device
} joystick;

static volatile uint8_t active = 255;
static joystick devices[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Virtual Device Driver ------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	devices[ref].dhi[comp] = DEFAULT_HANDLER;

	uint8_t reg1_len;
	switch (devices[ref].mode) {
		case MODE_FIREBIRD:
			reg1_len = 3;
			break;
		case MODE_MOUSESTICK:
			reg1_len = 2;
			break;
		default:
			return;
	}

	computer_data_set(comp, devices[ref].drv_idx, 1,
			devices[ref].reg1, reg1_len, true);
}

static void drvr_switch(uint8_t comp)
{
	active = comp;
}

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = devices[ref].dhi[comp];
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl == 0x01
			|| hndl == 0x23
			|| hndl == 0x4E) {
		devices[ref].dhi[comp] = hndl;

		if (hndl == 0x4E) {
			uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x00 };
			computer_data_set(comp, devices[ref].drv_idx, 0,
					data, sizeof(data), false);
		}
	}
}

static dev_driver joystick_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = NULL,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_get_handle,
	.set_handle_func = drvr_set_handle
};

/*
 * ----------------------------------------------------------------------------
 * --- Handler for the Physical Device ----------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t, bool))
{
	if (device_count >= MAX_DEVICES) return false;
	if (info->address_def != DEFAULT_ADDRESS) return false;

	// try supported device handlers
	uint8_t handle = 0;
	if (handle_change(0x4E, true)) {
		// Firebird/Blackhawk
		handle = 0x4E;
	} else if (handle_change(0x23, true)) {
		// Mousestick II
		handle = 0x23;
	} else {
		// unsupported device
		return false;
	}

	// setup storage
	joystick *dev = &devices[device_count];
	dev->hdev = info->hdev;

	// read register 1 to determine specific model info
	uint8_t dev_reg1[8];
	uint8_t dev_reg1_len;
	host_sync_cmd(info->hdev, COMMAND_TALK_1, dev_reg1, &dev_reg1_len);
	switch (handle) {
		case 0x4E:
			if (dev_reg1_len == 3) {
				// allow all possible variants reported by device
				memcpy(dev->reg1, dev_reg1, 3);
				dev->mode = MODE_FIREBIRD;
			} else {
				// not valid extended response, reset to original handler
				dbg_err("gjoy: dev %d dhid 0x4e bad reg1", info->hdev);
				handle_change(info->dhid_def, true);
				return false;
			}
			break;
		case 0x23:
			if (dev_reg1_len == 2 && dev_reg1[0] == 0x04) {
				// only support 0x0400 for 3-byte for the moment
				memcpy(dev->reg1, dev_reg1, 2);
				dev->mode = MODE_MOUSESTICK;
			} else {
				// not valid extended response, reset to original handler
				dbg_err("gjoy: dev %d dhid 0x23 bad reg1", info->hdev);
				handle_change(info->dhid_def, true);
				return false;
			}
			break;
		default:
			dbg_err("gjoy: coding error reg1 %d", handle);
			return false;
	}

	// for emulation, start at device handler 1 until changed
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		dev->dhi[c] = DEFAULT_HANDLER;
	}

	driver_register(&dev->drv_idx, &joystick_driver, device_count);
	device_count++;
	return true;
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	// ignore until there is a computer to send data to
	if (active >= COMPUTER_COUNT) return;

	// select the correct device mapping
	uint8_t i;
	for (i = 0; i < device_count; i++) {
		if (devices[i].hdev == hdev) break;
	}
	if (i == device_count) return;

	// we only use register 0
	if (reg != 0) {
		dbg_err("gjoy: reg %d, ignoring", reg);
		return;
	}

	// verify input data is satisfactory
	switch (devices[i].mode) {
		case MODE_FIREBIRD:
			if (data_len != 8) {
				dbg_err("gjoy: r0 len != 8, %d", data_len);
				return;
			}
			break;
		case MODE_MOUSESTICK:
			// TODO this needs to be reworked for 0x0300 support in the future
			if (data_len != 3) {
				dbg_err("gjoy: r0 len != 3, %d", data_len);
				return;
			}
			break;
		default:
			dbg_err("gjoy: r0 talk bug");
			return;
	}

	// remap data from the real device to the virtual handler
	uint8_t odata[8];
	uint8_t odata_len;
	switch (devices[i].dhi[active]) {
		case 0x4E:
			if (devices[i].mode == MODE_FIREBIRD) {
				// already in correct format
				memcpy(odata, data, data_len);
				odata_len = data_len;
			} else if (devices[i].mode == MODE_MOUSESTICK) {
				// translate from Firebird->Mousestick
				// TODO IMPLEMENT
				dbg("gjoy: unimplemented");
				return;
			} else {
				dbg_err("gjoy: remap bug!");
				return;
			}
			break;
		case 0x23:
			if (devices[i].mode == MODE_FIREBIRD) {
				// translate from Mousestick->Firebird
				// TODO IMPLEMENT
				dbg("gjoy: unimplemented");
				return;
			} else if (devices[i].mode == MODE_MOUSESTICK) {
				// already in correct format
				memcpy(odata, data, data_len);
				odata_len = data_len;
			} else {
				dbg_err("gjoy: remap bug!");
				return;
			}
			break;
		default:
			bool button;
			int8_t x, y;
			if (devices[i].mode == MODE_FIREBIRD) {
				// translate from Firebird->Mouse
				button = false;
				if (data[0] != 0xFF || data[1] != 0xFF || data[2] != 0xFF) {
					button = true;
				}
				x = (data[3] - 0x80);
				y = (data[4] - 0x80);
			} else if (devices[i].mode == MODE_MOUSESTICK) {
				// translate from Mousestick->Mouse
				button = false;
				if (data[2] != 0xFF) {
					button = true;
				}
				x = (data[0] - 0x80);
				y = (data[1] - 0x80);
			} else {
				dbg_err("gjoy: remap bug!");
				return;
			}
			// decrease mouse movement radius to avoid wild pointer behavior
			x /= 8;
			y /= 8;
			// store as mouse movement
			odata[0] = (y & 0x7F) | (button ? 0x00 : 0x80);
			odata[1] = (x & 0x7F) | 0x80;
			odata_len = 2;

			dbg("gjoy: %d %d", odata[0], odata[1]);
	}

	// store input data directly into the response registers
	computer_data_set(active, devices[i].drv_idx, 0, odata, odata_len, false);
}

static ndev_handler joystick_handler = {
	.name = "gjoy",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void joystick_init(void)
{
	handler_register(&joystick_handler);
}
