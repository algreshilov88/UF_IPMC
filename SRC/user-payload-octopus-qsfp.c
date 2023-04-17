/*
    UF_IPMC/user-payload.c

    Copyright (C) 2020 Aleksei Greshilov
    aleksei.greshilov@cern.ch

    UF_IPMC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    UF_IPMC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with UF_IPMC.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "string.h"
#include "ipmi.h"
#include "picmg.h"
#include "event.h"
#include "ipmc.h"
#include "i2c.h"
#include "i2c-sensor.h"
#include "event.h"
#include "debug.h"
#include "timer.h"
#include "toml.h"
#include "sensor.h"
#include "logger.h"
#include "user-payload-octopus-qsfpdd.h"
#include "user-sensor-octopus-qsfpdd.h"
#include "semaphore.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include "octopus.h"

#define PAYLOAD_OFF		0
#define PAYLOAD_ON		1


#define qbv_on_off              0x1220000

extern unsigned long long int lbolt;
extern FRU_CACHE fru_inventory_cache[];
extern unsigned char current_sensor_count;
extern SDR_ENTRY sdr_entry_table[];
extern int i2c_fd_snsr[];

unsigned long long int payload_timeout_init = 0;
static int power_up_done = 0;

void user_module_payload_on( void )
{
	unsigned int payload_read;

	lock(1);
	payload_read = reg_read(devmem_ptr, qbv_on_off);
	payload_read |= 0x20;
	reg_write(devmem_ptr, qbv_on_off, payload_read);
	//printf("Octopus done\n");
	payload_timeout_init = lbolt;
	power_up_octopus(i2c_fd_snsr[1]);
	logger("PAYLOAD", "On");
	power_up_done = 1;
	printf("lbolt = %llu, payload = %llu\n",lbolt,payload_timeout_init);
	unlock(1);

}

void
user_module_payload_off( void )
{
  lock(1);
  unsigned int payload_read;
  payload_read = reg_read(devmem_ptr, qbv_on_off);
  payload_read &= ~0x20;
  power_down_octopus(i2c_fd_snsr[1]);
  power_up_done = 0;
  reg_write(devmem_ptr, qbv_on_off, payload_read);
  logger("PAYLOAD", "Off");
  unlock(1);

}

void
payload_state_poll( unsigned char *arg )
{
	toml_table_t* config;
	toml_table_t* pld_m;
	toml_raw_t raw;
	long long int man_payload_status;
	long long int man_payload;
	unsigned char payload_poll_timer_handle;
	unsigned int payload_state;
	unsigned int man_payload_state;
	unsigned int payload_read;

	FILE* fp;
	char errbuf[1000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r");
		logger("ERROR", "fopen() in payload_state_poll() (CONFIG.toml)");
	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in payload_state_poll() (CONFIG.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [PAYLOAD_M] table. */
	if (0 == (pld_m = toml_table_in(config, "PAYLOAD_M"))) {
		logger("ERROR", "toml_table_in() 'PAYLOAD_M' in payload_state_poll()");
	}

	/* Extract 'man_handle_status' config value. */
	if (0 == (raw = toml_raw_in(pld_m, "man_payload_status"))) {
		logger("ERROR", "toml_raw_in() 'man_payload_status' in payload_state_poll()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &man_payload_status)) {
		logger("ERROR", "toml_rtoi() 'man_payload_status' in payload_state_poll()");
	}

	/* Extract 'man_payload' config value. */
	if (0 == (raw = toml_raw_in(pld_m, "man_payload"))) {
		logger("ERROR", "toml_raw_in() 'man_payload' in payload_state_poll()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &man_payload)) {
		logger("ERROR", "toml_rtoi() 'man_payload' in payload_state_poll()");
	}

	if (1 == (unsigned char) man_payload_status) {
		man_payload_state = (unsigned int) man_payload;
		payload_read = reg_read(devmem_ptr, qbv_on_off);
		payload_state = (payload_read >> 5)&0x01;

        	if (payload_state != man_payload_state) {
                	switch (man_payload_state) {
                        	case PAYLOAD_ON:
                                	user_module_payload_on();
                                	break;
                        	case PAYLOAD_OFF:
                                	user_module_payload_off();
                                	break;
			}
		}
	}

	toml_free(config);

	// Re-start the timer
	timer_add_callout_queue( (void *)&payload_poll_timer_handle,
		       	1*SEC, payload_state_poll, 0 ); /* 1 sec timeout */
}

int check_power_up( void ) {
    return power_up_done;
}
