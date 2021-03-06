/*
HydraBus/HydraNFC - Copyright (C) 2014 Benjamin VERNOUX

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include "hydrabus_mode_spi.h"
#include "xatoi.h"
#include "bsp_spi.h"
#include <string.h>

static const char* str_pins_spi1= { "SPI1 CS=PA15, SCK=PB3, MISO=PB4, MOSI=PB5" };
static const char* str_pins_spi2= { "SPI2 CS=PC1(SW), SCK=PB10, MISO=PC2, MOSI=PC3" };
static const char* str_name_spi= { "SPI" };
static const char* str_prompt_spi1= { "spi1> " };
static const char* str_prompt_spi2= { "spi2> " };

const mode_exec_t mode_spi_exec = {
	.mode_cmd          = &mode_cmd_spi,       /* Terminal parameters specific to this mode */
	.mode_start        = &mode_start_spi,     /* Start command '[' */
	.mode_startR       = &mode_startR_spi,    /* Start Read command '{' */
	.mode_stop         = &mode_stop_spi,      /* Stop command ']' */
	.mode_stopR        = &mode_stopR_spi,     /* Stop Read command '}' */
	.mode_write        = &mode_write_spi,     /* Write/Send 1 data */
	.mode_read         = &mode_read_spi,      /* Read 1 data command 'r' */
	.mode_write_read   = &mode_write_read_spi,/* Write & Read 1 data implicitely with mode_write command */
	.mode_clkh         = &mode_clkh_spi,      /* Set CLK High (x-WIRE or other raw mode ...) command '/' */
	.mode_clkl         = &mode_clkl_spi,      /* Set CLK Low (x-WIRE or other raw mode ...) command '\' */
	.mode_dath         = &mode_dath_spi,      /* Set DAT High (x-WIRE or other raw mode ...) command '-' */
	.mode_datl         = &mode_datl_spi,      /* Set DAT Low (x-WIRE or other raw mode ...) command '_' */
	.mode_dats         = &mode_dats_spi,      /* Read Bit (x-WIRE or other raw mode ...) command '!' */
	.mode_clk          = &mode_clk_spi,       /* CLK Tick (x-WIRE or other raw mode ...) command '^' */
	.mode_bitr         = &mode_bitr_spi,      /* DAT Read (x-WIRE or other raw mode ...) command '.' */
	.mode_periodic     = &mode_periodic_spi,  /* Periodic service called (like UART sniffer...) */
	.mode_macro        = &mode_macro_spi,     /* Macro command "(x)", "(0)" List current macros */
	.mode_setup        = &mode_setup_spi,     /* Configure the device internal params with user parameters (before Power On) */
	.mode_setup_exc    = &mode_setup_exc_spi, /* Configure the physical device after Power On (command 'W') */
	.mode_cleanup      = &mode_cleanup_spi,   /* Exit mode, disable device enter safe mode SPI... */
	.mode_print_param    = &mode_print_param_spi,    /* Print Mode parameters */
	.mode_print_pins     = &mode_print_pins_spi,     /* Print Pins used */
	.mode_print_settings = &mode_print_settings_spi, /* Settings string */
	.mode_print_name     = &mode_print_name_spi,      /* Print Mode name */
	.mode_str_prompt   = &mode_str_prompt_spi    /* Prompt name string */
};

static const char* str_dev_arg_num[]= {
	"Choose SPI device number: 1=SPI1, 2=SPI2\r\n"
};
static const char* str_dev_param_num[]= {
	"1=SPI1",
	"2=SPI2"
};

static const char* str_dev_param_gpio_pull[]= {
	"1=SCK/MISO/MOSI NoPull",
	"2=SCK/MISO/MOSI PullUp",
	"3=SCK/MISO/MOSI PullDown"
};
static const char* str_dev_arg_gpio_pull[]= {
	"Choose SPI SCK/MISO/MOSI Pull(~40Kohm) mode:\r\n1=NoPull(External Pull), 2=PullUp, 3=PullDown(Common)\r\n"
};

static const char* str_dev_param_mode[]= {
	"1=Slave",
	"2=Master"
};
static const char* str_dev_arg_mode[]= {
	"Choose SPI Mode: 1=Slave, 2=Master\r\n"
};

static const char* str_dev_param_speed[2][8]= {
	/* SPI1 */
	{
		/* 0  */ "1=0.32MHz",
		/* 1  */ "2=0.65MHz",
		/* 2  */ "3=1.31MHz",
		/* 3  */ "4=2.62MHz",
		/* 4  */ "5=5.25MHz",
		/* 5  */ "6=10.5MHz",
		/* 6  */ "7=21MHz",
		/* 7  */ "8=42MHz"
	},
	/* SPI2 */
	{
		/* 0  */ "1=0.16MHz",
		/* 1  */ "2=0.32MHz",
		/* 2  */ "3=0.65MHz",
		/* 3  */ "4=1.31MHz",
		/* 4  */ "5=2.62MHz",
		/* 5  */ "6=5.25MHz",
		/* 6  */ "7=10.5MHz",
		/* 7  */ "8=21MHz"
	}
};
static const char* str_dev_arg_speed[]= {
	"Choose SPI1 Freq:\r\n1=0.32MHz, 2=0.65MHz, 3=1.31MHz, 4=2.62MHz,\r\n5=5.25MHz, 6=10.5MHz, 7=21MHz, 8=42MHz\r\n",
	"Choose SPI2 Freq:\r\n1=0.16MHz, 2=0.32MHz, 3=0.65MHz, 4=1.31MHz,\r\n5=2.62MHz, 6=5.25MHz, 7=10.5MHz, 8=21MHz\r\n"
};

static const char* str_dev_param_cpol_cpha[]= {
	"1=POL0/PHA0",
	"2=POL0/PHA1",
	"3=POL1/PHA0",
	"4=POL1/PHA1"
};
static const char* str_dev_arg_cpol_cpha[]= {
	"Choose SPI Clock Polarity/Phase:\r\n1=POL0/PHA0, 2=POL0/PHA1, 3=POL1/PHA0, 4=POL1/PHA1\r\n"
};

static const char* str_dev_param_bit_lsb_msb[]= {
	"1=MSB Tx first",
	"2=LSB Tx first"
};
static const char* str_dev_arg_bit_lsb_msb[]= {
	"Choose SPI LSB/MSB Transmitted First:\r\n1=MSB Tx first, 2=LSB Tx first\r\n"
};

/*
TODO SPI Number of bits mode
static const char* str_dev_numbits[]={
 "Choose SPI Number of bits\r\n1=8 bits, 2=16 bits\r\n" };
*/

static const mode_dev_arg_t mode_dev_arg[] = {
	/* argv0 */ { .min=1, .max=2, .dec_val=TRUE, .param=DEV_NUM, .argc_help=ARRAY_SIZE(str_dev_arg_num), .argv_help=str_dev_arg_num },
	/* argv1 */ { .min=1, .max=3, .dec_val=TRUE, .param=DEV_GPIO_PULL, .argc_help=ARRAY_SIZE(str_dev_arg_gpio_pull), .argv_help=str_dev_arg_gpio_pull },
	/* argv2 */ { .min=1, .max=2, .dec_val=TRUE, .param=DEV_MODE, .argc_help=ARRAY_SIZE(str_dev_arg_mode), .argv_help=str_dev_arg_mode },
	/* argv3 */ { .min=1, .max=8, .dec_val=TRUE, .param=DEV_SPEED, .argc_help=ARRAY_SIZE(str_dev_arg_speed), .argv_help=str_dev_arg_speed },
	/* argv4 */ { .min=1, .max=4, .dec_val=TRUE, .param=DEV_CPOL_CPHA, .argc_help=ARRAY_SIZE(str_dev_arg_cpol_cpha), .argv_help=str_dev_arg_cpol_cpha },
	/* argv5 */ { .min=1, .max=2, .dec_val=TRUE, .param=DEV_BIT_LSB_MSB, .argc_help=ARRAY_SIZE(str_dev_arg_bit_lsb_msb), .argv_help=str_dev_arg_bit_lsb_msb }
// { .min=1, .max=2, .dec_val=TRUE,.param=DEV_NUMBITS, .argc_help=ARRAY_SIZE(str_dev_numbits), .argv_help=str_dev_numbits },

};
#define MODE_DEV_NB_ARGC ((int)ARRAY_SIZE(mode_dev_arg)) /* Number of arguments/parameters for this mode */

/* Terminal parameters management specific to this mode */
/* Return TRUE if success else FALSE */
bool mode_cmd_spi(t_hydra_console *con, int argc, const char* const* argv)
{
	long dev_val;
	int arg_no;

	if(argc == 0) {
		hydrabus_mode_dev_manage_arg(con, 0, NULL, 0, 0, (mode_dev_arg_t*)&mode_dev_arg);
		return FALSE;
	}

	/* Ignore additional parameters */
	if(argc > MODE_DEV_NB_ARGC)
		argc = MODE_DEV_NB_ARGC;

	for(arg_no = 0; arg_no < argc; arg_no++) {
		dev_val = hydrabus_mode_dev_manage_arg(con, argc, argv, MODE_DEV_NB_ARGC, arg_no, (mode_dev_arg_t*)&mode_dev_arg);
		if(dev_val == HYDRABUS_MODE_DEV_INVALID) {
			return FALSE;
		}
	}

	if(argc == MODE_DEV_NB_ARGC) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/* Start command '[' */
void mode_start_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	bsp_spi_select(proto->dev_num);
	cprintf(con, hydrabus_mode_str_cs_enabled);
}

/* Start Read command '{' */
void mode_startR_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	bsp_spi_select(proto->dev_num);
	cprintf(con, hydrabus_mode_str_cs_enabled);
}

/* Stop command ']' */
void mode_stop_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	bsp_spi_unselect(proto->dev_num);
	cprintf(con, hydrabus_mode_str_cs_disabled);
}

/* Stop Read command '}' */
void mode_stopR_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	bsp_spi_unselect(proto->dev_num);
	cprintf(con, hydrabus_mode_str_cs_disabled);
}

/* Write/Send x data return status 0=OK */
uint32_t mode_write_spi(t_hydra_console *con, uint8_t *tx_data, uint8_t nb_data)
{
	int i;
	uint32_t status;
	mode_config_proto_t* proto = &con->mode->proto;

	status = bsp_spi_write_u8(proto->dev_num, tx_data, nb_data);
	if(status == BSP_OK) {
		if(nb_data == 1) {
			/* Write 1 data */
			cprintf(con, hydrabus_mode_str_write_one_u8, tx_data[0]);
		} else if(nb_data > 1) {
			/* Write n data */
			cprintf(con, hydrabus_mode_str_mul_write);
			for(i = 0; i < nb_data; i++) {
				cprintf(con, hydrabus_mode_str_mul_value_u8, tx_data[i]);
			}
			cprintf(con, hydrabus_mode_str_mul_br);
		}
	}
	return status;
}

/* Read x data command 'r' return status 0=OK */
uint32_t mode_read_spi(t_hydra_console *con, uint8_t *rx_data, uint8_t nb_data)
{
	int i;
	uint32_t status;
	mode_config_proto_t* proto = &con->mode->proto;

	status = bsp_spi_read_u8(proto->dev_num, rx_data, nb_data);
	if(status == BSP_OK) {
		if(nb_data == 1) {
			/* Read 1 data */
			cprintf(con, hydrabus_mode_str_read_one_u8, rx_data[0]);
		} else if(nb_data > 1) {
			/* Read n data */
			cprintf(con, hydrabus_mode_str_mul_read);
			for(i = 0; i < nb_data; i++) {
				cprintf(con, hydrabus_mode_str_mul_value_u8, rx_data[i]);
			}
			cprintf(con, hydrabus_mode_str_mul_br);
		}
	}
	return status;
}

/* Write & Read x data return status 0=OK */
uint32_t mode_write_read_spi(t_hydra_console *con, uint8_t *tx_data, uint8_t *rx_data, uint8_t nb_data)
{
	int i;
	uint32_t status;
	mode_config_proto_t* proto = &con->mode->proto;

	status = bsp_spi_write_read_u8(proto->dev_num, tx_data, rx_data, nb_data);
	if(status == BSP_OK) {
		if(nb_data == 1) {
			/* Write & Read 1 data */
			cprintf(con, hydrabus_mode_str_write_read_u8, tx_data[0], rx_data[0]);
		} else if(nb_data > 1) {
			/* Write & Read n data */
			for(i = 0; i < nb_data; i++) {
				cprintf(con, hydrabus_mode_str_write_read_u8, tx_data[i], rx_data[i]);
			}
		}
	}
	return status;
}

/* Set CLK High (x-WIRE or other raw mode ...) command '/' */
void mode_clkh_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* Set CLK Low (x-WIRE or other raw mode ...) command '\' */
void mode_clkl_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* Set DAT High (x-WIRE or other raw mode ...) command '-' */
void mode_dath_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* Set DAT Low (x-WIRE or other raw mode ...) command '_' */
void mode_datl_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* Read Bit (x-WIRE or other raw mode ...) command '!' */
void mode_dats_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* CLK Tick (x-WIRE or other raw mode ...) command '^' */
void mode_clk_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* DAT Read (x-WIRE or other raw mode ...) command '.' */
void mode_bitr_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* Periodic service called (like UART sniffer...) */
uint32_t mode_periodic_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
	return 0;
}

/* Macro command "(x)", "(0)" List current macros */
void mode_macro_spi(t_hydra_console *con, uint32_t macro_num)
{
	(void)con;
	(void)macro_num;
	/* TODO mode_spi Macro command "(x)" */
}

/* Configure the device internal params with user parameters (before Power On) */
void mode_setup_spi(t_hydra_console *con)
{
	(void)con;
	/* Nothing to do in SPI mode */
}

/* Configure the physical device after Power On (command 'W') */
void mode_setup_exc_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	bsp_spi_init(proto->dev_num, proto);
}

/* Exit mode, disable device safe mode SPI... */
void mode_cleanup_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	bsp_spi_deinit(proto->dev_num);
}

/* Mode parameters string (does not include m & bus_mode) */
void mode_print_param_spi(t_hydra_console *con)
{

	cprintf(con, "%d %d %d %d %d %d",
		con->mode->proto.dev_num+1,
		con->mode->proto.dev_gpio_pull+1,
		con->mode->proto.dev_mode+1,
		con->mode->proto.dev_speed+1,
		con->mode->proto.dev_cpol_cpha+1,
		con->mode->proto.dev_bit_lsb_msb+1);
}

/* Print pins used */
void mode_print_pins_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	if(proto->dev_num == 0)
		cprint(con, str_pins_spi1, strlen(str_pins_spi1));
	else
		cprint(con, str_pins_spi2, strlen(str_pins_spi2));
}

/* Print settings */
void mode_print_settings_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	cprintf(con, "Device: %s\r\nGPIO Pull: %s\r\nMode: %s\r\nSpeed: %s\r\nClock Polarity/Phase: %s\r\nBit LSB/MSB: %s",
		str_dev_param_num[proto->dev_num],
		str_dev_param_gpio_pull[proto->dev_gpio_pull],
		str_dev_param_mode[proto->dev_mode],
		str_dev_param_speed[proto->dev_num][proto->dev_speed],
		str_dev_param_cpol_cpha[proto->dev_cpol_cpha],
		str_dev_param_bit_lsb_msb[proto->dev_bit_lsb_msb]);
}

/* Print mode name */
void mode_print_name_spi(t_hydra_console *con)
{

	cprint(con, str_name_spi, strlen(str_name_spi));
}

/* Return Prompt name */
const char* mode_str_prompt_spi(t_hydra_console *con)
{
	mode_config_proto_t* proto = &con->mode->proto;

	if(proto->dev_num == 0) {
		return str_prompt_spi1;
	} else
		return str_prompt_spi2;
}
