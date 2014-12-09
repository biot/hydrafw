/*
 * HydraBus/HydraNFC
 *
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

enum {
	T_HELP = 1,
	T_HISTORY,
	T_EXIT,
	T_CLEAR,
	T_DEBUG,
	T_SHOW,
	T_SYSTEM,
	T_MEMORY,
	T_THREADS,
	T_SD,
	T_MOUNT,
	T_UMOUNT,
	T_CD,
	T_PWD,
	T_LS,
	T_CAT,
	T_HD,
	T_ERASE,
	T_REALLY,
	T_TESTPERF,
	T_MODE,
	T_SPI,
	T_I2C,
	T_DEVICE,
	T_MASTER,
	T_SLAVE,
	T_FREQUENCY,
	T_POLARITY,
	T_PHASE,
	T_MSB_FIRST,
	T_LSB_FIRST,
	T_PULL,
	T_UP,
	T_DOWN,
	T_FLOATING,
	T_ON,
	T_OFF,
	T_CHIP_SELECT,
	T_CS,
	T_PINS,
	T_READ,
	T_WRITE,
	T_START,
	T_STOP,
	T_UART,
	T_SPEED,
	T_PARITY,
	T_NONE,
	T_EVEN,
	T_ODD,
	T_STOP_BITS,
	T_ADC,
	T_ADC1,
	T_TEMPSENSOR,
	T_VREFINT,
	T_VBAT,
	T_SAMPLES,
	T_PERIOD,
	T_CONTINUOUS,
	T_NFC,
	T_MIFARE,
	T_VICINITY,
	T_REGISTERS,
	T_SCAN,
	T_SNIFF,
	T_GPIO,
	T_IN,
	T_OUT,
	T_OPEN_DRAIN,
	T_TOKENLINE,
	T_TIMING,
	T_RM,
	T_MKDIR,
	T_LOGGING,

	/* BP-compatible commands */
	T_LEFT_SQ,
	T_RIGHT_SQ,
	T_LEFT_CURLY,
	T_RIGHT_CURLY,
	T_SLASH,
	T_BACKSLASH,
	T_MINUS,
	T_UNDERSCORE,
	T_EXCLAMATION,
	T_CARET,
	T_DOT,
	T_AMPERSAND,
	T_PERCENT,
};

