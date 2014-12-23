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

#include "tokenline.h"
#include "commands.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

t_token_dict tl_dict[] = {
	{ /* Dummy entry */ },
	{ T_HELP, "help" },
	{ T_HISTORY, "history" },
	{ T_EXIT, "exit" },
	{ T_CLEAR, "clear" },
	{ T_DEBUG, "debug" },
	{ T_SHOW, "show" },
	{ T_SYSTEM, "system" },
	{ T_MEMORY, "memory" },
	{ T_THREADS, "threads" },
	{ T_SD, "sd" },
	{ T_MOUNT, "mount" },
	{ T_UMOUNT, "umount" },
	{ T_CD, "cd" },
	{ T_PWD, "pwd" },
	{ T_LS, "ls" },
	{ T_CAT, "cat" },
	{ T_HD, "hd" },
	{ T_ERASE, "erase" },
	{ T_REALLY, "really" },
	{ T_TESTPERF, "test-perf" },
	{ T_MODE, "mode" },
	{ T_SPI, "spi" },
	{ T_I2C, "i2c" },
	{ T_DEVICE, "device" },
	{ T_MASTER, "master" },
	{ T_SLAVE, "slave" },
	{ T_FREQUENCY, "frequency" },
	{ T_POLARITY, "polarity" },
	{ T_PHASE, "phase" },
	{ T_MSB_FIRST, "msb-first" },
	{ T_LSB_FIRST, "lsb-first" },
	{ T_PULL, "pull" },
	{ T_UP, "up" },
	{ T_DOWN, "down" },
	{ T_FLOATING, "floating" },
	{ T_ON, "on" },
	{ T_OFF, "off" },
	{ T_CS_ON, "cs-on" },
	{ T_CS_OFF, "cs-off" },
	{ T_PINS, "pins" },
	{ T_READ, "read" },
	{ T_WRITE, "write" },
	{ T_START, "start" },
	{ T_STOP, "stop" },
	{ T_UART, "uart" },
	{ T_SPEED, "speed" },
	{ T_PARITY, "parity" },
	{ T_NONE, "none" },
	{ T_EVEN, "even" },
	{ T_ODD, "odd" },
	{ T_STOP_BITS, "stop-bits" },
	{ T_ADC, "adc" },
	{ T_ADC1, "adc1" },
	{ T_TEMPSENSOR, "tempsensor" },
	{ T_VREFINT, "vrefint" },
	{ T_VBAT, "vbat" },
	{ T_SAMPLES, "samples" },
	{ T_PERIOD, "period" },
	{ T_CONTINUOUS, "continuous" },
	{ T_NFC, "nfc" },
	{ T_MIFARE, "mifare" },
	{ T_VICINITY, "vicinity" },
	{ T_REGISTERS, "registers" },
	{ T_SCAN, "scan" },
	{ T_SNIFF, "sniff" },
	{ T_GPIO, "gpio" },
	{ T_IN, "in" },
	{ T_OUT, "out" },
	{ T_OPEN_DRAIN, "open-drain" },
	{ T_TOKENLINE, "tokenline" },
	{ T_TIMING, "timing" },
	{ T_RM, "rm" },
	{ T_MKDIR, "mkdir" },
	{ T_LOGGING, "logging" },

	{ T_LEFT_SQ, "[" },
	{ T_RIGHT_SQ, "]" },
	{ T_LEFT_CURLY, "{" },
	{ T_RIGHT_CURLY, "}" },
	{ T_SLASH, "/" },
	{ T_BACKSLASH, "\\" },
	{ T_MINUS, "-" },
	{ T_UNDERSCORE, "_" },
	{ T_EXCLAMATION, "!" },
	{ T_CARET, "^" },
	{ T_DOT, "." },
	{ T_AMPERSAND, "&" },
	{ T_PERCENT, "%" },
	{ }
};

t_token tokens_on_off[] = {
	{ T_ON },
	{ T_OFF },
	{ }
};

t_token tokens_master_slave[] = {
	{ T_MASTER },
	{ T_SLAVE },
	{ }
};

t_token tokens_gpio_pull[] = {
	{
		T_UP,
		.help = "Pull up"
	},
	{
		T_DOWN,
		.help = "Pull down"
	},
	{
		T_FLOATING,
		.help = "Floating (default)"
	},
	{ }
};

t_token tokens_mode_show[] = {
	{
		T_PINS,
		.help = "Show pins used in this mode"
	},
	{ }
};

t_token tokens_mode_nfc_show[] = {
	{
		T_REGISTERS,
		.help = "Show NFC registers"
	},
	{ }
};

t_token tokens_mode_nfc_scan[] = {
	{
		T_PERIOD,
		.arg_type = T_ARG_INT,
		.help = "Delay between scans (msec)"
	},
	{
		T_CONTINUOUS,
		.help = "Scan until interrupted"
	},
	{ }
};

t_token tokens_mode_nfc[] = {
	{
		T_SHOW,
		.subtokens = tokens_mode_nfc_show,
		.help = "Show NFC parameters"
	},
	{
		T_MIFARE,
		.help = "MIFARE (ISO14443A)"
	},
	{
		T_VICINITY,
		.help = "Vicinity (ISO/IEC 15693)"
	},
	{
		T_SCAN,
		.subtokens = tokens_mode_nfc_scan,
		.help = "Scan"
	},
	{
		T_SNIFF,
		.help = "Sniff (ISO14443A only)"
	},
	{
		T_EXIT,
		.help = "Exit NFC mode"
	},
	{ }
};

t_token tokens_nfc[] = {
	{
		T_MIFARE,
		.help = "MIFARE (ISO14443A)"
	},
	{
		T_VICINITY,
		.help = "Vicinity (ISO/IEC 15693)"
	},
	{ }
};

t_token tokens_parity[] = {
	{ T_NONE },
	{ T_EVEN },
	{ T_ODD },
};

t_token tokens_mode_uart[] = {
	{
		T_SHOW,
		.subtokens = tokens_mode_show,
		.help = "Show UART parameters"
	},
	{
		T_DEVICE,
		.arg_type = T_ARG_INT,
		.help = "UART device (1 or 2)"
	},
	{
		T_SPEED,
		.arg_type = T_ARG_INT,
		.help = "Bitrate"
	},
	{
		T_PARITY,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_parity,
		.help = "Parity (none/even/odd)"
	},
	{
		T_STOP_BITS,
		.arg_type = T_ARG_INT,
		.help = "Stop bits (1 or 2)"
	},
	/* UART-specific commands */
	{
		T_READ,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Read byte (repeat with :<num>)"
	},
	{
		T_WRITE,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)"
	},
	{
		T_ARG_INT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)"
	},
	/* BP commands */
	{
		T_AMPERSAND,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 usec (repeat with :<num>)"
	},
	{
		T_PERCENT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 msec (repeat with :<num>)"
	},
	{
		T_EXIT,
		.help = "Exit UART mode"
	},
	{ }
};

t_token tokens_uart[] = {
	{
		T_DEVICE,
		.arg_type = T_ARG_INT,
		.help = "UART device (1 or 2)"
	},
	{
		T_SPEED,
		.arg_type = T_ARG_INT,
		.help = "Bitrate"
	},
	{
		T_PARITY,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_parity,
		.help = "Parity (none/even/odd)"
	},
	{
		T_STOP_BITS,
		.arg_type = T_ARG_INT,
		.help = "Stop bits (1 or 2)"
	},
	{ }
};

t_token tokens_mode_i2c[] = {
	{
		T_SHOW,
		.subtokens = tokens_mode_show,
		.help = "Show I2C parameters"
	},
	{
		T_PULL,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_gpio_pull,
		.help = "GPIO pull"
	},
	{
		T_FREQUENCY,
		.arg_type = T_ARG_FREQ,
		.help = "Read/write frequency"
	},
	/* I2C-specific commands */
	{
		T_START,
		.help = "Start"
	},
	{
		T_STOP,
		.help = "Stop"
	},
	{
		T_READ,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Read byte (repeat with :<num>)"
	},
	{
		T_WRITE,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)"
	},
	{
		T_ARG_INT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)"
	},
	{
		T_SCAN,
		.help = "Scan for connected devices"
	},
	/* BP commands */
	{
		T_LEFT_SQ,
		.help = "Alias for \"start\""
	},
	{
		T_RIGHT_SQ,
		.help = "Alias for \"stop\""
	},
	{
		T_AMPERSAND,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 usec (repeat with :<num>)"
	},
	{
		T_PERCENT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 msec (repeat with :<num>)"
	},
	{
		T_EXIT,
		.help = "Exit SPI mode"
	},
	{ }
};

t_token tokens_i2c[] = {
	{
		T_PULL,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_gpio_pull,
		.help = "GPIO pull"
	},
	{
		T_FREQUENCY,
		.arg_type = T_ARG_FREQ,
		.help = "Read/write frequency"
	},
	{ }
};

#define SPI_PARAMETERS \
	{ T_DEVICE, \
		.arg_type = T_ARG_INT, \
		.help = "SPI device (1 or 2)" }, \
	{ T_PULL, \
		.arg_type = T_ARG_TOKEN, \
		.subtokens = tokens_gpio_pull, \
		.help = "GPIO pull" }, \
	{ T_MODE, \
		.arg_type = T_ARG_TOKEN, \
		.subtokens = tokens_master_slave, \
			.help = "Master or slave mode" }, \
	{ T_FREQUENCY, \
		.arg_type = T_ARG_FREQ, \
		.help = "Read/write frequency" }, \
	{ T_POLARITY, \
		.arg_type = T_ARG_INT, \
		.help = "Clock polarity (0 or 1)" }, \
	{ T_PHASE, \
		.arg_type = T_ARG_INT, \
		.help = "Clock phase (0 or 1)" }, \
	{ T_MSB_FIRST, \
		.help = "Send/receive MSB first" }, \
	{ T_LSB_FIRST, \
		.help = "Send/receive LSB first" },

t_token tokens_mode_spi[] = {
	{
		T_SHOW,
		.subtokens = tokens_mode_show,
		.help = "Show SPI parameters"
	},
	SPI_PARAMETERS
	/* SPI-specific commands */
	{
		T_READ,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Read byte (repeat with :<num>)"
	},
	{
		T_WRITE,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)"
	},
	{
		T_ARG_INT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Write byte (repeat with :<num>)"
	},
	{
		T_CS_ON,
		.help = "Alias for \"chip-select on\""
	},
	{
		T_CS_OFF,
		.help = "Alias for \"chip-select off\""
	},
	/* BP commands */
	{
		T_LEFT_SQ,
		.help = "Alias for \"chip-select on\""
	},
	{
		T_RIGHT_SQ,
		.help = "Alias for \"chip-select off\""
	},
	{
		T_AMPERSAND,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 usec (repeat with :<num>)"
	},
	{
		T_PERCENT,
		.flags = T_FLAG_SUFFIX_TOKEN_DELIM_INT,
		.help = "Delay 1 msec (repeat with :<num>)"
	},
	{
		T_EXIT,
		.help = "Exit SPI mode"
	},
	{ }
};

t_token tokens_spi[] = {
	SPI_PARAMETERS
	{ }
};

t_token tokens_gpio_mode[] = {
	{
		T_IN,
		.help = "Configure GPIO as input"
	},
	{
		T_OUT,
		.help = "Configure GPIO as output"
	},
	{
		T_OPEN_DRAIN,
		.help = "Configure GPIO pin as open drain (default)"
	},
	{ }
};

t_token tokens_gpio[] = {
	{
		T_ARG_STRING,
		.help = "One or more GPIO pins (PA0-15, PB0-11, PC0-15, PA* ...)"
	},
	{
		T_MODE,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_gpio_mode,
		.help = "Configure as input, output or open drain"
	},
	{
		T_PULL,
		.arg_type = T_ARG_TOKEN,
		.subtokens = tokens_gpio_pull,
		.help = "GPIO pull up, down or floating"
	},
	{
		T_PERIOD,
		.arg_type = T_ARG_INT,
		.help = "Delay between reads, in milliseconds"
	},
	{
		T_CONTINUOUS,
		.help = "Read continuously"
	},
	{
		T_ON,
		.help = "Set GPIO pin"
	},
	{
		T_OFF,
		.help = "Clear GPIO pin"
	},
	{
		T_READ,
		.help = "Read GPIO values"
	},
	{ }
};

t_token tokens_adc[] = {
	{
		T_ADC1,
		.help = "ADC1 (PA1)"
	},
	{
		T_TEMPSENSOR,
		.help = "Temperature sensor"
	},
	{
		T_VREFINT,
		.help = "Internal reference voltage"
	},
	{
		T_VBAT,
		.help = "VBAT voltage"
	},
	{
		T_SAMPLES,
		.arg_type = T_ARG_INT,
		.help = "Number of samples"
	},
	{
		T_PERIOD,
		.arg_type = T_ARG_INT,
		.help = "Delay between samples (msec)"
	},
	{
		T_CONTINUOUS,
		.help = "Read continuously"
	},
	{ }
};

t_token tokens_really[] = {
	{ T_REALLY },
	{ }
};

t_token tokens_sd[] = {
	{
		T_MOUNT,
		.help = "Mount SD card"
	},
	{
		T_UMOUNT,
		.help = "Unmount SD card"
	},
	{
		T_ERASE,
		.subtokens = tokens_really,
		.help = "Erase and reformat SD card"
	},
	{
		T_CD,
		.arg_type = T_ARG_STRING,
		.help = "Change SD card directory"
	},
	{
		T_PWD,
		.help = "Show current SD card directory"
	},
	{
		T_LS,
		.help = "List files on SD card"
	},
	{
		T_TESTPERF,
		.help = "Test SD card performance"
	},
	{
		T_CAT,
		.arg_type = T_ARG_STRING,
		.help = "Display (ASCII) file on SD card"
	},
	{
		T_HD,
		.arg_type = T_ARG_STRING,
		.help = "Hexdump file on SD card"
	},
	{
		T_RM,
		.arg_type = T_ARG_STRING,
		.help = "Remove file or directory"
	},
	{
		T_MKDIR,
		.arg_type = T_ARG_STRING,
		.help = "Create new directory"
	},
	{ }
};

t_token tokens_show[] = {
	{ T_SYSTEM },
	{ T_MEMORY },
	{ T_THREADS },
	{ T_SD },
	{ T_DEBUG },
	{ }
};

t_token tokens_debug[] = {
	{
		T_TOKENLINE,
		.help = "Tokenline dump for every command"
	},
	{
		T_TIMING,
		.help = "Mysterious timing function"
	},
	{
		T_ON,
		.help = "Enable"
	},
	{
		T_OFF,
		.help = "Disable"
	},
	{ }
};

t_token tokens_logging[] = {
	{ T_SD,
		.arg_type = T_ARG_STRING,
		.help = "Log to file on SD card" },
	{ T_ON,
		.help = "Start logging" },
	{ T_OFF,
		.help = "Stop logging" },
	{ }
};

t_token tl_tokens[] = {
	{
		T_HELP,
		.arg_type = T_ARG_HELP,
		.help = "Available commands"
	},
	{
		T_HISTORY,
		.help = "Command history"
	},
	{
		T_CLEAR,
		.help = "Clear screen"
	},
	{
		T_SHOW,
		.subtokens = tokens_show,
		.help = "Show information"
	},
	{
		T_LOGGING,
		.subtokens = tokens_logging,
		.help = "Turn logging on or off"
	},
	{
		T_SD,
		.subtokens = tokens_sd,
		.help = "SD card management"
	},
	{
		T_ADC,
		.subtokens = tokens_adc,
		.help = "Read analog values"
	},
	{
		T_GPIO,
		.subtokens = tokens_gpio,
		.help = "Get or set GPIO pins"
	},
	{
		T_SPI,
		.subtokens = tokens_spi,
		.help = "SPI mode"
	},
	{
		T_I2C,
		.subtokens = tokens_i2c,
		.help = "I2C mode"
	},
	{
		T_UART,
		.subtokens = tokens_uart,
		.help = "UART mode"
	},
	{
		T_NFC,
		.subtokens = tokens_nfc,
		.help = "NFC mode"
	},
	{
		T_DEBUG,
		.subtokens = tokens_debug,
		.help = "Debug mode"
	},
	{ }
};

#pragma GCC diagnostic pop

