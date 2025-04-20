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

#include "kensington.h"

#define DEFAULT_ADDRESS       3
#define DEFAULT_PRI_HANDLER   0x32
#define DEFAULT_SEC_HANDLER   0x01
#define REGISTER_1_LEN        8
#define REGISTER_2_LEN        7

// sets the most number of passthru mice permitted
#define MAX_DEVICES 2

typedef enum {
	KMODE_PRI_100CPI = 0,
	KMODE_PRI_200CPI,
	KMODE_PRI_EXTENDED
} kmode;

typedef struct {
	uint8_t hdev;
	uint8_t drv_idx_pri, drv_idx_sec;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	bool pending;      // true if motion cache is valid
	int16_t x, y;      // accumulated X/Y movement data
	uint8_t buttons;   // last seen button data, 0=pressed, 1=released
	bool extended;     // device responds on register 1 for extended protocol
	uint8_t reg1_default[REGISTER_1_LEN];
	uint8_t reg2_default[REGISTER_2_LEN];
	uint8_t reg2_flags[COMPUTER_COUNT];
} kmouse;

static volatile uint8_t active = 255;
static kmouse mice[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Utility Functions ------------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool reg2_check(volatile uint8_t* data)
{
	uint8_t c = 0;
	for (uint8_t i = 0; i < REGISTER_2_LEN; i++) {
		c = ~(c ^ data[i]);
	}
	return c == data[REGISTER_2_LEN];
}

// handles storing register 2 information for altering device behavior
static void reg2_apply(uint8_t comp, uint8_t dev, uint8_t* data)
{
	mice[dev].reg2_flags[comp] = data[0];
}

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Driver ---------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	// reset primary device registers
	computer_data_set(comp, mice[ref].drv_idx_pri, 2,
			mice[ref].reg2_default, REGISTER_2_LEN, true);
	if (mice[ref].extended) {
		computer_data_set(comp, mice[ref].drv_idx_pri, 1,
				mice[ref].reg1_default, REGISTER_1_LEN, true);
	}

	// and any local storage for reference
	mice[ref].reg2_flags[comp] = mice[ref].reg2_default[0];

	// reset secondary handler and clear register 1 response
	mice[ref].dhi[comp] = DEFAULT_SEC_HANDLER;
	computer_data_set(comp, mice[ref].drv_idx_sec, 1, NULL, 0, false);

	// clear motion data if the current computer is in control
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

static void drvr_pri_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = DEFAULT_PRI_HANDLER;
}

static void drvr_sec_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	if (mice[ref].reg2_flags[comp] & 0x80) {
		// primary is active, do not reply
		*hndl = 0xFF;
	} else {
		// secondary is active, give back handler
		*hndl = mice[ref].dhi[comp];
	}
}

static void drvr_sec_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl == 0x01 || hndl == 0x02) {
		mice[ref].dhi[comp] = hndl;
	} else if (mice[ref].extended && hndl == 0x04) {
		mice[ref].dhi[comp] = hndl;
		computer_data_set(comp, mice[ref].drv_idx_sec, 1,
				mice[ref].reg1_default, REGISTER_1_LEN, true);
	}
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg, bool pri)
{
	if (active != comp) return;

	uint8_t drv_idx = pri ? mice[ref].drv_idx_pri : mice[ref].drv_idx_sec;
	bool extended = mice[ref].extended
			&& (pri ? true : mice[ref].dhi[comp] == 0x04);

	if (reg == 0 && xSemaphoreTake(mice[ref].sem, portMAX_DELAY)) {
		if (mice[ref].pending) {
			uint8_t data[5];
			util_mouse_encode(data, mice[ref].x, mice[ref].y, mice[ref].buttons);
			uint8_t len = extended ? 3 : 2;
			if (computer_data_offer(active, drv_idx, 0, data, len)) {
				mice[ref].pending = false;
			}
		}
		xSemaphoreGive(mice[ref].sem);
	}
}

static void drvr_pri_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	drvr_talk(comp, ref, reg, true);
}

static void drvr_sec_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	drvr_talk(comp, ref, reg, false);
}

static void drvr_pri_listen(uint8_t comp, uint32_t ref, uint8_t reg,
		volatile uint8_t* data, uint8_t data_len)
{
	if (reg != 2) return;
	if (data_len != 8) return;
	if (! reg2_check(data)) return;

	// construct new register 2 data block
	uint8_t reg2[REGISTER_2_LEN];
	reg2[0] = data[0];
	reg2[1] = data[1];
	reg2[2] = mice[ref].reg2_default[2];
	reg2[3] = mice[ref].reg2_default[3];
	reg2[4] = data[4];
	reg2[5] = data[5];
	reg2[6] = data[6];

	// store relevant data that affects local device behavior
	reg2_apply(comp, ref, reg2);

	// store the updated register information for talking back
	computer_data_set(comp, mice[ref].drv_idx_pri, 2,
			reg2, REGISTER_2_LEN, true);
}

static dev_driver primary_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_pri_talk,
	.listen_func = drvr_pri_listen,
	.flush_func = NULL, // TODO is issued, uncertain impact on device
	.get_handle_func = drvr_pri_get_handle,
	.set_handle_func = NULL
};

static dev_driver secondary_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_sec_talk,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_sec_get_handle,
	.set_handle_func = drvr_sec_set_handle
};

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real Device ------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t, bool))
{
	if (device_count >= MAX_DEVICES) return false;
	if (info->address_def != DEFAULT_ADDRESS) return false;
	if (info->dhid_cur != DEFAULT_PRI_HANDLER) return false;

	// attempt to read required register 2 data
	host_err err = HOSTERR_OK;
	uint8_t tmp[8];
	uint8_t tmp_len;
	err = host_sync_cmd(info->hdev, COMMAND_TALK_2, tmp, &tmp_len);
	if (err != HOSTERR_OK || tmp_len != REGISTER_2_LEN) {
		dbg_err("kensington: dev %d bad reg2 err:%d", info->hdev, err);
		return false;
	}

	// setup device data
	kmouse *mse = &mice[device_count];
	mse->hdev = info->hdev;
	if (mse->sem == NULL) {
		mse->sem = xSemaphoreCreateMutex();
	}
	assert(mse->sem != NULL);
	mse->buttons = 0xFF;
	memcpy(mse->reg2_default, tmp, REGISTER_2_LEN);

	// activate the device
	tmp[0] = 0xA5;
	tmp[1] = 0x14;
	tmp[2] = 0x00;
	tmp[3] = 0x00;
	tmp[4] = 0x69;
	tmp[5] = 0xFF;
	tmp[6] = 0xFF;
	tmp[7] = 0x27;
	tmp_len = 8;
	err = host_sync_cmd(info->hdev, COMMAND_LISTEN_2, tmp, &tmp_len);
	if (err != HOSTERR_OK) {
		dbg_err("kensington: dev %d can't activate err:%d", info->hdev, err);
		return false;
	}

	// attempt to read register 1 data
	err = host_sync_cmd(info->hdev, COMMAND_TALK_1, tmp, &tmp_len);
	if (err == HOSTERR_OK) {
		if (tmp_len == 8) {
			memcpy(mse->reg1_default, tmp, 8);
			mse->extended = true;
		} else {
			dbg_err("kensington: dev %d bad reg1 len:", info->hdev, tmp_len);
			return false;
		}
	} else if (err == HOSTERR_TIMEOUT) {
		// expected with older devices
		mse->extended = false;
	} else {
		dbg_err("kensington: dev %d bad reg1 err:%d", info->hdev, err);
		return false;
	}

	// for secondary emulation, start at device handler 0x01 until changed
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mse->dhi[c] = DEFAULT_SEC_HANDLER;
	}

	driver_register(&mse->drv_idx_pri, &primary_driver, device_count);
	driver_register(&mse->drv_idx_sec, &secondary_driver, device_count);

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
		if (mice[i].hdev == hdev) break;
	}
	if (i == device_count) return;

	if (reg == 0 && data_len >= 2) {
		// decode incoming data
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

			// figure out where to send it and what format is expected
			uint8_t drv_idx;
			uint8_t data_out_len;
			if (mice[i].reg2_flags[active] & 0x80) {
				// primary
				drv_idx = mice[i].drv_idx_pri;
				data_out_len = mice[i].extended ? 3 : 2;
			} else {
				// secondary
				drv_idx = mice[i].drv_idx_sec;
				data_out_len = mice[i].dhi[active] == 0x04 ? 3 : 2;
			}

			// try to send data, or if send can't be done, store
			if (computer_data_offer(active, drv_idx, 0,
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

		dbg("kensington: %d %d %d", data[0], data[1], data[2]);
	}
}

static ndev_handler kensington_handler = {
	.name = "kensington",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void kensington_init(void)
{
	handler_register(&kensington_handler);
}
