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

#include "microrl_config.h"
#include <string.h>
#include <stdlib.h>

#include "common.h"
#include "microsd.h"
#include "microrl.h"
#include "microrl_common.h"
#include "microrl_callback.h"

#include "hydrabus.h"
#include "hydrabus_microrl.h"
#include "hydrabus_mode.h"

// definition commands word
#define _CMD_HELP0        "?"
#define _CMD_HELP1        "h"
#define _CMD_CLEAR        "clear"
#define _CMD_INFO         "info"

#define _CMD_CH_MEM       "ch_mem"
#define _CMD_CH_THREADS   "ch_threads"

#define _CMD_SD_MOUNT     "mount"
#define _CMD_SD_UMOUNT    "umount"
#define _CMD_SD_CD        "cd"
#define _CMD_SD_PWD       "pwd"
#define _CMD_SD_LS        "ls"
#define _CMD_SD_CAT       "cat"
#define _CMD_SD_HD        "hd"
#define _CMD_SD_ERASE     "erase"
#define _CMD_SD_RPERFO    "sd_rperfo"

/* Update hydrabus_microrl.h => HYDRABUS_NUM_OF_CMD if new command are added/removed */
microrl_exec_t hydrabus_keyworld[HYDRABUS_NUM_OF_CMD] = {
	/* 0  */ { _CMD_HELP0,       &hydrabus_print_help },
	/* 1  */ { _CMD_HELP1,       &hydrabus_print_help },
	/* 2  */ { _CMD_CLEAR,       &print_clear },
	/* 3  */ { _CMD_INFO,        &cmd_info },
	/* 4  */ { _CMD_CH_MEM,      &cmd_mem },
	/* 5  */ { _CMD_CH_THREADS,  &cmd_threads },
	/* 6  */ { _CMD_SD_MOUNT,    &cmd_sd_mount },
	/* 7  */ { _CMD_SD_UMOUNT,   &cmd_sd_umount },
	/* 8  */ { _CMD_SD_CD,       &cmd_sd_cd },
	/* 9  */ { _CMD_SD_PWD,      &cmd_sd_pwd },
	/* 10 */ { _CMD_SD_LS,       &cmd_sd_ls },
	/* 11 */ { _CMD_SD_CAT,      &cmd_sd_cat },
	/* 12 */ { _CMD_SD_HD,       &cmd_sd_cat},
	/* 13 */ { _CMD_SD_ERASE,    &cmd_sd_erase},
	/* 14 */ { _CMD_SD_RPERFO,   &cmd_sd_read_perfo },
	/* 15 */ { _HYDRABUS_MODE,      &hydrabus_mode },
	/* 16 */ { _HYDRABUS_MODE_INFO, &hydrabus_mode_info }
};

// array for completion
char* hydrabus_compl_world[HYDRABUS_NUM_OF_CMD + 1];

//*****************************************************************************
void hydrabus_print_help(t_hydra_console *con, int argc, const char* const* argv)
{
	(void)argc;
	(void)argv;

	print(con, "Use TAB key for completion\n\r");
	print(con, "? or h         - Help\t\n\r");
	print(con, "clear          - clear screen\n\r");
	print(con, "info           - info on FW & HW\n\r");
	print(con, "ch_mem         - memory info\t\n\r");
	print(con, "ch_threads     - threads\n\r");
	print(con, "mount          - mount sd\n\r");
	print(con, "umount         - unmount sd\n\r");
	print(con, "cd <dir>       - change directory in sd\n\r");
	print(con, "pwd            - show current directory path in sd\n\r");
	print(con, "ls [opt dir]   - list files in sd\n\r");
	print(con, "cat <filename> - display sd file (ASCII)\n\r");
	print(con, "hd <filename>  - hexdump sd file\n\r");
	print(con, "sd_rperfo      - sd read performance test\n\r");
	print(con, "erase          - erase sd\n\r");

	print(con, "\n\r");
	print(con, "m              - Change mode\n\r");
	print(con, "i              - Mode information\n\r");
	print(con, "Protocol Interaction\n\r");
	print(con, "----------------------------------------\n\r");
	//print(con, "(x)\t\tMacro x\n\r");
	print(con, "[ Start\t\t] Stop\n\r");
	//print(con, "{ Start+read\t} Stop+read\n\r");
	//print(con, "/ CLK hi\t\\ CLK lo\n\r");
	//print(con, "^ CLK tick\n\r");
	//print(con, "- DAT hi\t_ DAT lo\n\r");
	//print(con, ". DAT read\t! Bit read\n\r");
	print(con, ": Repeat (e.g. r:10)\n\r");
	//print(con, "\"abc\"\t\tSend string\n\r");
	print(con, "& DELAY us\t% DELAY ms\n\r");
	print(con, "123 0x12 0b110\tSend 8bits val\n\r");
	print(con, "r Read\n\r");
}

//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int hydrabus_execute(t_hydra_console *con, int argc, const char* const* argv)
{
	bool cmd_found;
	int curr_arg = 0;
	int cmd;

	// just iterate through argv word and compare it with your commands
	cmd_found = FALSE;
	curr_arg = 0;
	while(curr_arg < argc) {
		cmd=0;
		while(cmd < HYDRABUS_NUM_OF_CMD) {
			if( (strcmp(argv[curr_arg], hydrabus_keyworld[cmd].str_cmd)) == 0 ) {
				hydrabus_keyworld[cmd].ptFunc_exe_cmd(con, argc-curr_arg, &argv[curr_arg]);
				cmd_found = TRUE;
				break;
			}
			cmd++;
		}
		if(cmd_found == FALSE) {
			if(hydrabus_mode_proto_inter(con, argc-curr_arg, &argv[curr_arg]) == FALSE) {
				print(con,"command: '");
				print(con,(char*)argv[curr_arg]);
				print(con,"' Error/Not found.\n\r");
			}
		}
		curr_arg++;
	}
	return 0;
}

//*****************************************************************************
void hydrabus_sigint(t_hydra_console *con)
{
	/* Hit ctrl-U and enter. */
	microrl_insert_char(con->mrl, 0x15);
	microrl_insert_char(con->mrl, 0x0d);
	microrl_insert_char(con->mrl, 0x0a);
}
