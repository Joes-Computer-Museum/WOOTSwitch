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

const baud = 115200;
const vid = 0x1209; // pid.codes
const pid = 0x6804; // hootswitch PID from pid.codes

const polynomial = 0x04C11DB7;
const configLength = 4096;
const writeCommand = 0xF3;
const encodeBase = 0x30;

/*
 * ----------------------------------------------------------------------------
 *   Interface Parsing
 * ----------------------------------------------------------------------------
 */

function parseSettings(arr)
{
	if (document.getElementById('cfg-buzzer').checked) {
		arr[0] &= ~0x01;
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Flash Commit Logic
 * ----------------------------------------------------------------------------
 */

function insertFrameCheck(arr)
{
	const revPolynomial = crc32_reverse(polynomial);
	const table = crc32_generate(revPolynomial);

	let crc = crc32_initial();
	let i;
	for (i = 0; i < configLength - 4; i++) {
		crc = crc32_add_byte(table, crc, arr[i]);
	}
	crc = crc32_reverse(crc);

	arr[i++] = (crc >>> 24) & 0xFF;
	arr[i++] = (crc >>> 16) & 0xFF;
	arr[i++] = (crc >>> 8) & 0xFF;
	arr[i++] = (crc >>> 0) & 0xFF;

	console.log("crc: " + (crc >>> 0).toString(16));
}

function flashConfig()
{
	if (! port) return;
	if (! window.confirm("This will write a new device configuration with the selected settings."
			+ " Do you want to continue?")) {
		return;
	}

	console.log("config flash requested");

	// make the storage array; 0xFF is default over the valid portion of the
	// array to match the default (unprogrammed) flash value of the device
	const arr = new Uint8Array(configLength * 2);
	for (let i = 0; i < configLength; i++) {
		arr[i] = 0xFF;
	}

	// load settings and then 'sign' them with the frame check
	parseSettings(arr);
	insertFrameCheck(arr);

	// rewrite for wire transmission
	for (let i = configLength - 1; i >= 0; i--) {
		let v = arr[i];
		arr[i * 2] = encodeBase + ((v >>> 4) & 0xF);
		arr[i * 2 + 1] = encodeBase + ((v >>> 0) & 0xF);
	}

	// transmit to the device
	const cmd = new Uint8Array([writeCommand]);
	writer.write(cmd);
	writer.write(arr);
}

/*
 * ----------------------------------------------------------------------------
 *   Serial Port Management and Logging
 * ----------------------------------------------------------------------------
 */

let port = undefined;
let reader = undefined;
let writer = undefined;
let log = "";

const connectButton = document.getElementById("connect-button");
const controlArea = document.getElementById("control-area");
const disconnectMessage = document.getElementById("disconnect-message");
const textLog = document.querySelector("textarea");
const decoder = new TextDecoder();

async function connect()
{
	if (! ("serial" in navigator)) {
		alert("Web Serial API is not supported by this browser. Try using Chrome/Chromium instead.");
		return;
	}
	if (port) {
		console.log("Ignoring connect(), already connected!");
		return;
	}

	// request port and connect to it
	try {
		port = await navigator.serial.requestPort({
			filters: [{
				usbVendorId: vid,
				usbProductId: pid
				}]
		});
		await port.open({
			baudRate: baud
		});
		reader = port.readable.getReader();
		writer = port.writable.getWriter();
		connectButton.classList.add("hidden");
		log += "[[ Connected! ]]\n";
		textLog.innerHTML = log;
		controlArea.classList.remove("hidden");
	} catch (err) {
		console.log(err);
		if (err.name != "NotFoundError") {
			alert(err);
		}
		return;
	}

	// repeatedly print contents to log until closed
	try {
		while (port) {
			const { value, done } = await reader.read();
			if (done) {
				break;
			}
			log += decoder.decode(value);
			textLog.innerHTML = log;
			textLog.scrollTop = textLog.scrollHeight;
		}
	} catch (err) {
		console.log(err);
		alert(err);
	} finally {
		if (reader) {
			reader.releaseLock();
			reader = undefined;
		}
		if (writer) {
			writer.releaseLock();
			writer = undefined;
		}

		port = undefined;
		log += "[[ Disconnected! ]]\n";
		textLog.innerHTML = log;
		controlArea.classList.add("hidden");
		disconnectMessage.classList.remove("hidden");
	}
}

async function disconnect()
{
	if (! port) {
		console.log("already disconnected, ignoring disconnected()");
		return;
	}

	// unset, let connect() finish up loop
	port = undefined;
	if (reader) {
		reader.cancel();
	}
}

connectButton.addEventListener("click", async () => {
	if (! port) {
		connect();
	} else {
		disconnect();
	}
});

const aboutButton = document.getElementById("about-button");
const aboutArea = document.getElementById("about-area");

aboutButton.addEventListener("click", () => {
	if (aboutArea.classList.contains("hidden")) {
		aboutArea.classList.remove("hidden");
	} else {
		aboutArea.classList.add("hidden");
	}
});
