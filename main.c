/*
 * HydraBus/HydraNFC
 *
 * Copyright (C) 2012-2014 Benjamin VERNOUX
 * Copyright (C) 2014 Bert Vermeulen <bert@biot.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include <stdio.h> /* sprintf */
#include "ch.h"
#include "hal.h"

#include "common.h"

#include "usb1cfg.h"
#include "usb2cfg.h"

#include "microsd.h"
#include "hydrabus.h"

/* USB1: Virtual serial port over USB.*/
SerialUSBDriver SDU1;
/* USB2: Virtual serial port over USB.*/
SerialUSBDriver SDU2;

extern t_token tl_tokens[];
extern t_token_dict tl_dict[];
extern char log_dest[];

// create tokenline objects for each console
t_tokenline tl_con1;
t_mode_config mode_con1 = { .proto={ .valid=MODE_CONFIG_PROTO_VALID, .bus_mode=MODE_CONFIG_PROTO_DEV_DEF_VAL }, .cmd={ 0 } };

t_tokenline tl_con2;
t_mode_config mode_con2 = { .proto={ .valid=MODE_CONFIG_PROTO_VALID, .bus_mode=MODE_CONFIG_PROTO_DEV_DEF_VAL }, .cmd={ 0 } };

t_hydra_console consoles[] = {
	{ .thread_name="console USB1", .sdu=&SDU1, .tl=&tl_con1, .mode = &mode_con1 },
	{ .thread_name="console USB2", .sdu=&SDU2, .tl=&tl_con2, .mode = &mode_con2 }
};

THD_FUNCTION(console, arg)
{
	t_hydra_console *con;

	con = arg;
	chRegSetThreadName(con->thread_name);
	tl_init(con->tl, tl_tokens, tl_dict, print, con);
	tl_set_prompt(con->tl, PROMPT);
	tl_set_callback(con->tl, execute);

	while (1) {
		tl_input(con->tl, get_char(con));
		chThdSleepMilliseconds(1);
	}
}

/*
 * Application entry point.
 */
#define BLINK_FAST   50
#define BLINK_SLOW   250

int div_test(int a, int b)
{
	return a / b;
}
volatile int a, b, c;

static t_hydra_console *find_console(SerialUSBDriver *sdu)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(consoles); i++) {
		if (consoles[i].sdu == sdu)
			return &consoles[i];
	}

	return NULL;
}

static size_t con_write(void *ip, const uint8_t *bp, size_t n)
{
	t_hydra_console *con;

	con = find_console(ip);
	if (!con)
		return n;
	if (*log_dest) {
		log_add(con, (char *)bp, n);
	}

	return con->orig_vmt->write(ip, bp, n);
}

static size_t con_writet(void *ip, const uint8_t *bp, size_t n, systime_t timeout)
{
	t_hydra_console *con;

	con = find_console(ip);
	if (!con)
		return n;
	if (*log_dest) {
		log_add(con, (char *)bp, n);
	}

	return con->orig_vmt->writet(ip, bp, n, timeout);
}

static msg_t con_put(void *ip, uint8_t b)
{
	t_hydra_console *con;

	con = find_console(ip);
	if (!con)
		return MSG_OK;
	if (*log_dest) {
		log_add(con, (char *)&b, 1);
	}

	return con->orig_vmt->put(ip, b);
}

static msg_t con_putt(void *ip, uint8_t b, systime_t timeout)
{
	t_hydra_console *con;

	con = find_console(ip);
	if (!con)
		return MSG_OK;
	if (*log_dest) {
		log_add(con, (char *)&b, 1);
	}

	return con->orig_vmt->putt(ip, b, timeout);
}

static void patch_console(t_hydra_console *con)
{
	con->orig_vmt = con->sdu->vmt;
	memcpy(&con->log_vmt, con->sdu->vmt, sizeof(struct SerialUSBDriverVMT));
	con->sdu->vmt = &con->log_vmt;
	con->log_vmt.write = con_write;
	con->log_vmt.writet = con_writet;
	con->log_vmt.put = con_put;
	con->log_vmt.putt = con_putt;
}

int main(void)
{
	unsigned int i;
	int sleep_ms;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device
	 *   drivers and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	scs_dwt_cycle_counter_enabled();

	hydrabus_init();

	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusb1cfg);

	sduObjectInit(&SDU2);
	sduStart(&SDU2, &serusb2cfg);

	/*
	 * Activates the USB1 & 2 driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusb1cfg.usbp);
	usbDisconnectBus(serusb2cfg.usbp);

	chThdSleepMilliseconds(500);

	usbStart(serusb1cfg.usbp, &usb1cfg);
	/*
	 * Disable VBUS sensing on USB1 (GPIOA9) is not connected to VUSB
	 * by default)
	 */
#define GCCFG_NOVBUSSENS (1U<<21)
	stm32_otg_t *otgp = (&USBD1)->otg;
	otgp->GCCFG |= GCCFG_NOVBUSSENS;

	usbConnectBus(serusb1cfg.usbp);

	usbStart(serusb2cfg.usbp, &usb2cfg);
	usbConnectBus(serusb2cfg.usbp);

	/* Wait for USB Enumeration. */
	chThdSleepMilliseconds(100);

	for (i = 0; i < ARRAY_SIZE(consoles); i++)
		patch_console(&consoles[i]);

	/*
	 * Normal main() thread activity.
	 */
	chRegSetThreadName("main");
	while (TRUE) {
		for (i = 0; i < ARRAY_SIZE(consoles); i++) {
			if (!consoles[i].thread) {
				if (consoles[i].sdu->config->usbp->state != USB_ACTIVE)
					continue;
				/* Spawn new console thread.*/
				consoles[i].thread = chThdCreateFromHeap(NULL,
						     CONSOLE_WA_SIZE, NORMALPRIO, console, &consoles[i]);
			} else {
				if (chThdTerminatedX(consoles[i].thread))
					/* This console thread terminated. */
					consoles[i].thread = NULL;
			}
		}

		/* For test purpose HydraBus ULED blink */
		if (USER_BUTTON)
			sleep_ms = BLINK_FAST;
		else
			sleep_ms = BLINK_SLOW;
		ULED_ON;

		chThdSleepMilliseconds(sleep_ms);

		if (USER_BUTTON) {
			sleep_ms = BLINK_FAST;
			/*
			{
				SCB->CCR |= 0x10;

				a = 100;
				b = 0;
				c = div_test(a, b);

			}
			return c;
			*/
		} else
			sleep_ms = BLINK_SLOW;
		ULED_OFF;

		chThdSleepMilliseconds(sleep_ms);
	}
}
