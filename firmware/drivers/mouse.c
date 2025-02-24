/*
 * Copyright (C) 2024 saybur
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
#include "semphr.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "handler.h"
#include "hardware.h"
#include "host.h"
#include "host_err.h"
#include "host_sync.h"
#include "util.h"

#include "mouse.h"

/*
 * Mouse driver for ADB relative motion devices following either the standard
 * (0x01/0x02) or extended (0x04) mouse protocols. See Technote HW01 (Space
 * Aliens Ate My Mouse) for protocol details and reference materials.
 *
 * The "classic" protocol uses handlers 0x01 (100cpi) or 0x02 (200cpi). For
 * simplicity only the former is used: if a computer switches the mouse to
 * 200cpi the data from a non-extended mouse is simply scaled when responding.
 */

#define DEFAULT_ADDRESS 3
#define DEFAULT_HANDLER 1

// sets the most number of passthru mice permitted
#define MAX_MICE 4

typedef struct {
	uint8_t hdev;
	uint8_t drv_idx;
	uint8_t dhi[COMPUTER_COUNT];
	bool extended;     // true if hardware mouse is in extended (0x04) mode
	SemaphoreHandle_t sem;
	bool pending;      // true if motion cache is valid
	int16_t x, y;      // accumulated X/Y movement data
	uint8_t buttons;   // last seen button data, 0=pressed, 1=released
	uint8_t reg1[8];   // register 1 data
} mouse;

static volatile uint8_t active = 255;
static mouse mice[MAX_MICE];
static uint8_t mouse_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Mouse Driver ---------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	mice[ref].dhi[comp] = DEFAULT_HANDLER;

	if (active == comp) {
		if (xSemaphoreTake(mice[ref].sem, portMAX_DELAY)) {
			mice[ref].pending = false;
			xSemaphoreGive(mice[ref].sem);
		}
	}
}

static void drvr_switch(uint8_t comp)
{
	active = comp;
}

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = mice[ref].dhi[comp];
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl != 0x01 || hndl != 0x02 || hndl != 0x04) return;

	mice[ref].dhi[comp] = hndl;
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	if (active != comp) return;

	if (reg == 0x00 && xSemaphoreTake(mice[ref].sem, portMAX_DELAY)) {
		if (mice[ref].pending) {
			uint8_t data[5];
			util_mouse_encode(data, mice[ref].x, mice[ref].y, mice[ref].buttons);
			uint8_t len = (mice[ref].dhi[comp] == 0x04 ? 5 : 2);
			if (computer_data_offer(active, mice[ref].drv_idx, 0, data, len)) {
				mice[ref].pending = false;
			}
		}
		xSemaphoreGive(mice[ref].sem);

	} else if (reg == 0x01 && mice[ref].dhi[comp] == 0x04) {
		// register 1 only supported in extended mouse protocol
		computer_data_offer(active, mice[ref].drv_idx, 0x01, mice[ref].reg1, 8);
	}
}

static dev_driver mouse_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_talk,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_get_handle,
	.set_handle_func = NULL
};

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real ADB Mice ----------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t))
{
	if (mouse_count >= MAX_MICE) return false;
	if (info->address_def != DEFAULT_ADDRESS) return false;

	mouse *mse = &mice[mouse_count];
	mse->hdev = info->hdev;
	mse->sem = xSemaphoreCreateMutex();
	assert(mse->sem != NULL);

	// try to change the device to the extended mouse protocol
	if (handle_change(0x04)) {
		// read and store register 1
		uint8_t dev_reg1[8];
		uint8_t dev_reg1_len;
		host_sync_cmd(info->hdev, COMMAND_TALK_1, dev_reg1, &dev_reg1_len);
		if (dev_reg1_len == 8) {
			// store register 1 information
			memcpy(mse->reg1, dev_reg1, 8);
			mse->extended = true;
		} else {
			// not valid extended response, reset to original handler
			dbg_err("mse: dev %d dhid 4 bad reg1", info->hdev);
			handle_change(info->dhid_def);
		}
	}

	// make sure device is in a valid mode
	if (! (info->dhid_cur == 0x01 || info->dhid_cur == 0x04)) {
		// already tried extended, move to basic protocol
		if (! handle_change(0x01)) {
			// failed to accept, must not be a mouse?
			dbg_err("mse: dev %d reject dhid 1, dropped", info->hdev);
			return false;
		}
	}

	// for emulation, start at device handler 1 until changed
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mse->dhi[c] = DEFAULT_HANDLER;
	}

	driver_register(&mse->drv_idx, &mouse_driver, mouse_count);
	mouse_count++;
	return true;
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	// ignore until there is a computer to send data to
	if (active >= COMPUTER_COUNT) return;

	// select the correct device mapping
	uint8_t i;
	for (i = 0; i < mouse_count; i++) {
		if (mice[i].hdev == hdev) break;
	}
	if (i == mouse_count) return;

	if (reg == 0 && data_len >= 2) {
		// decode incoming data from the mouse
		int16_t xt, yt;
		uint8_t buttons;
		util_mouse_decode(data, data_len, &xt, &yt, &buttons);

		if (xSemaphoreTake(mice[i].sem, portMAX_DELAY)) {
			// include any pending motion data
			if (mice[i].pending) {
				xt += mice[i].x;
				yt += mice[i].y;
			}

			// encode the resulting output
			uint8_t data_out[5];
			util_mouse_encode(data_out, xt, yt, buttons);
			uint8_t data_out_len = mice[i].extended ? 5 : 2;

			// try to send data, or if send can't be done, store
			if (computer_data_offer(active, mice[i].drv_idx, 0,
					data_out, data_out_len)) {
				mice[i].pending = false;
			} else {
				mice[i].pending = true;
				mice[i].x = xt;
				mice[i].y = yt;
				mice[i].buttons = buttons;
			}
			xSemaphoreGive(mice[i].sem);
		}

		dbg("mse: %d %d", data[0], data[1]);
	}
}

static ndev_handler mouse_handler = {
	.name = "mse",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void mouse_init(void)
{
	handler_register(&mouse_handler);

	// by default register 1 reports as ADB Mouse II
	static const uint8_t default_reg1[8] =
		{0x40, 0x32, 0x30, 0x30, 0x00, 0xC8, 0x01, 0x01};
	for (uint8_t i = 0; i < MAX_MICE; i++) {
		mice[i].buttons = 0xFF;
		memcpy(mice[i].reg1, default_reg1, sizeof(default_reg1));
	}
}
