/*
    UF_IPMC/ipmc.c

    Original work Copyright (C) 2007-2008 Gokhan Sozmen
    http://www.coreipm.com

    Modified work Copyright 2020 Aleksei Greshilov
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
#include "ws.h"
#include "picmg.h"
#include "event.h"
#include "ipmc.h"
#include "i2c.h"
#include "i2c-sensor.h"
#include "event.h"
#include "debug.h"
#include "timer.h"
#include "ws.h"
#include "sensor.h"
#include "toml.h"
#include "logger.h"
#include "user-payload.h"
#include "user-sensor.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>

#define MAX_SDR_COUNT		20
#define MAX_SENSOR_COUNT	20

#define ha_offset		0x1220008
#define qbv_on_off		0x1220000

int wdt_fd;

unsigned int hot_swap_handle_last_state;
unsigned int fru_ipmb_a_b_last_state = 0x00;
unsigned int fru_ipmb_a_b_init_state = 0x00;
unsigned int fru_ipmb_a_b_event_set = 0x00;

extern unsigned long long int lbolt;
extern FRU_CACHE fru_inventory_cache[];
extern unsigned char current_sensor_count;
extern SDR_ENTRY sdr_entry_table[];
extern int i2c_fd_snsr[];

MGMT_CTRL_DEV_LOCATOR_RECORD sdr0;
FULL_SENSOR_RECORD sdr[MAX_SDR_COUNT];
SENSOR_DATA sd[MAX_SENSOR_COUNT];

// FRU info data
/*
struct fru_data {
	FRU_COMMON_HEADER hdr;
	FRU_INTERNAL_USE_AREA internal;
	FRU_CHASSIS_INFO_AREA_HDR chassis;
	BOARD_AREA_FORMAT_HDR board;
	PRODUCT_AREA_FORMAT_HDR product;
	MODULE_CURRENT_REQUIREMENTS_RECORD mcr;
} fru_data;
*/
struct fru_data {
	FRU_COMMON_HEADER hdr;
    /*unsigned char internal[72];
	unsigned char chassis[32];
	unsigned char board[64];
	unsigned char product[80];
    MULTIRECORD_AREA_HEADER last_record;*/
} fru_data;


/*==============================================================*/
/* Local Function Prototypes					*/
/*==============================================================*/

void module_init( void );
void switch_state_poll( unsigned char *arg );
void fru_data_init( void );
void module_sensor_init( void );
void payload_state_poll( unsigned char *arg );
void fru_hot_swap_state_poll( unsigned char *arg );
void ipmb_0_state_poll( unsigned char *arg);
void pok_state_poll( unsigned char *arg );
void watchdog_state_poll( unsigned char *arg );

unsigned char
module_get_i2c_address( int address_type )
{
	unsigned int addr;
	unsigned char i2c_addr;
	switch( address_type ) {
		case I2C_ADDRESS_LOCAL:
			addr = reg_read(devmem_ptr, ha_offset);
			i2c_addr = (unsigned char) (addr & 0x7f)*2;
      return i2c_addr;
			break;
		case I2C_ADDRESS_REMOTE:
			return 0x20;
			break;
		default:
			return 0;
	}
}

/*==============================================================
 * INITIALIZATION
 *==============================================================*/
 void
 watchdog_init( void )
 {
         // ====================================================================
         // initialize the Watchdog Timer
         // ====================================================================

	 /* set new timeout value 1000s
	 Note the value should be within [5, 1000] */
	 int timeout = 500;
	 
         /* open WDT0 device (WDT0 enables itself automatically) */
         wdt_fd = open("/dev/watchdog0", O_RDWR);
         if(wdt_fd < 0) {
                 logger("ERROR", "Open watchdog device failed!");
                 exit(EXIT_FAILURE);
         }
         else {printf("/dev/watchdog0 opened\n");}
	 
	 /* set timeout */
	 if (ioctl(wdt_fd, WDIOC_SETTIMEOUT, &timeout) < 0) {
           	logger("ioctl(WDIOC_SETTIMEOUT) in watchdog_init()", strerror(errno));
         }
	 else {printf("New watchdog timeout value set to %d seconds\n", timeout);}
	 
	 
	 /* check timeout */
	 if (ioctl(wdt_fd, WDIOC_GETTIMEOUT, &timeout) < 0) {
           	logger("ioctl(WDIOC_GETTIMEOUT) in watchdog_init()", strerror(errno));
         }
	 else {printf("Checked watchdog timeout value: %d seconds\n", timeout);}
 }

void
fru_data_init( void )
{
	// ====================================================================
	// initialize the FRU data records
	// ====================================================================
	toml_table_t* config;
	toml_table_t* FRU;
	toml_raw_t raw;
	long long int format_version;
	long long int int_use_offset;
	long long int chassis_info_offset;
	long long int board_offset;
	long long int product_info_offset;
	long long int multirecord_offset;
	long long int pad;

	FILE* fp;
	char errbuf[2000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/FRU/FRU.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/FRU/FRU.toml", "r");
		logger("ERROR", "fopen() in fru_data_init() (FRU.toml)");
	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in fru_data_init() (FRU.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [FRU] table. */
	if (0 == (FRU = toml_table_in(config, "FRU"))) {
		logger("ERROR", "toml_table_in() 'FRU' in fru_data_init()");
	}

	/* Extract 'format_version' config value. */
	if (0 == (raw = toml_raw_in(FRU, "format_version"))) {
		logger("ERROR", "toml_raw_in() 'format_version' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &format_version)) {
		logger("ERROR", "toml_rtoi() 'format_version' in fru_data_init()");
	}

	/* Extract 'int_use_offset' config value. */
	if (0 == (raw = toml_raw_in(FRU, "int_use_offset"))) {
		logger("ERROR", "toml_raw_in() 'int_use_offset' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &int_use_offset)) {
		logger("ERROR", "toml_rtoi() 'int_use_offset' in fru_data_init()");
	}

	/* Extract 'chassis_info_offset' config value. */
	if (0 == (raw = toml_raw_in(FRU, "chassis_info_offset"))) {
		logger("ERROR", "toml_raw_in() 'chassis_info_offset' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &chassis_info_offset)) {
		logger("ERROR", "toml_rtoi() 'chassis_info_offset' in fru_data_init()");
	}

	/* Extract 'board_offset' config value. */
	if (0 == (raw = toml_raw_in(FRU, "board_offset"))) {
		logger("ERROR", "toml_raw_in() 'board_offset' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &board_offset)) {
		logger("ERROR", "toml_rtoi() 'board_offset' in fru_data_init()");
	}

	/* Extract 'product_info_offset' config value. */
	if (0 == (raw = toml_raw_in(FRU, "product_info_offset"))) {
		logger("ERROR", "toml_raw_in() 'product_info_offset' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &product_info_offset)) {
		logger("ERROR", "toml_rtoi() 'product_info_offset' in fru_data_init()");
	}

	/* Extract 'multirecord_offset' config value. */
	if (0 == (raw = toml_raw_in(FRU, "multirecord_offset"))) {
		logger("ERROR", "toml_raw_in() 'multirecord_offset' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &multirecord_offset)) {
		logger("ERROR", "toml_rtoi() 'multirecord_offset' in fru_data_init()");
	}

	/* Extract 'pad' config value. */
	if (0 == (raw = toml_raw_in(FRU, "pad"))) {
		logger("ERROR", "toml_raw_in() 'pad' in fru_data_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &pad)) {
		logger("ERROR", "toml_rtoi() 'pad' in fru_data_init()");
	}

	fru_inventory_cache[0].fru_dev_id = 0;
	fru_inventory_cache[0].fru_inventory_area_size = sizeof( fru_data );
	fru_inventory_cache[0].fru_data = ( unsigned char * )( &fru_data );

	// FRU data header
	fru_data.hdr.format_version = (unsigned char) format_version;
	fru_data.hdr.int_use_offset = (unsigned char) int_use_offset;	// not used currently
	fru_data.hdr.chassis_info_offset = (unsigned char) chassis_info_offset;
	fru_data.hdr.board_offset = (unsigned char) board_offset;
	fru_data.hdr.product_info_offset = (unsigned char) product_info_offset;
	fru_data.hdr.multirecord_offset = (unsigned char) multirecord_offset;	//( ( char * )&( fru_data.p2p_rec ) - ( char * )&( fru_data ) ) >> 3;
	fru_data.hdr.pad = (unsigned char) pad;
	fru_data.hdr.checksum = ipmi_calculate_checksum( ( unsigned char * )&( fru_data.hdr ), sizeof( FRU_COMMON_HEADER ) - 1 );

	toml_free(config);
}

void
module_init( void )
{
	toml_table_t* config;
	toml_table_t* hdl_m;
	toml_raw_t raw;
	long long int man_handle_status;
	long long int man_handle_switch;
	unsigned char switch_poll_timer_handle;
	unsigned int handle_state;
	unsigned int hot_swap_sw;

	FILE* fp;
	char errbuf[1000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r");
		logger("ERROR", "fopen() in module_init() (CONFIG.toml)");
	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in module_init() (CONFIG.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [HANDLE_SWITCH_M] table. */
	if (0 == (hdl_m = toml_table_in(config, "HANDLE_SWITCH_M"))) {
		logger("ERROR", "toml_table_in() 'HANDLE_SWITCH_M' in module_init()");
	}

	/* Extract 'man_handle_status' config value. */
	if (0 == (raw = toml_raw_in(hdl_m, "man_handle_status"))) {
		logger("ERROR", "toml_raw_in() 'man_handle_status' in module_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &man_handle_status)) {
		logger("ERROR", "toml_rtoi() 'man_handle_status' in module_init()");
	}

	/* Extract 'man_handle_switch' config value. */
	if (0 == (raw = toml_raw_in(hdl_m, "man_handle_switch"))) {
		logger("ERROR", "toml_raw_in() 'man_handle_switch' in module_init()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &man_handle_switch)) {
		logger("ERROR", "toml_rtoi() 'man_handle_switch' in module_init()");
	}

	if (1 == (unsigned char) man_handle_status) {
			handle_state = (unsigned int) man_handle_switch;
			hot_swap_handle_last_state = handle_state;
	} else {
			hot_swap_sw = reg_read(devmem_ptr, ha_offset);
			handle_state = (hot_swap_sw >> 16)&0x01;
			hot_swap_handle_last_state = handle_state;
	}

	toml_free(config);

	// ====================================================================
	/* Turn on blue LED. When the Modules Management Power is enabled,
	 * the BLUE LED should turn on as soon as possible. */
//	gpio_led_on( GPIO_FRU_LED_BLUE );

	fru_data_init();

	fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M0_NOT_INSTALLED;

	picmg_m1_state( 0 );

	module_sensor_init();

	user_module_sensor_init();

	watchdog_init();

	/*==============================================================*/
	/* State Poll Functions call																		*/
	/*==============================================================*/

	payload_state_poll( 0 );

	fru_hot_swap_state_poll( 0 );

	ipmb_0_state_poll( 0 );

	watchdog_state_poll( 0 );

	user_sensor_state_poll();

	/*==============================================================*/
	/* I2C message handlers																					*/
	/*==============================================================*/

	ws_process_work_list( 0 );

	// ====================================================================
	// Handle current state of Hot Swap Handle

	if( handle_state == HANDLE_SWITCH_CLOSED )
	{
//			usleep(2000000);
			picmg_m2_state( 0 );
	}

	// ====================================================================
	// check the hot swap switch periodically
	timer_add_callout_queue( (void *)&switch_poll_timer_handle,
		       	0.1*SEC, switch_state_poll, 0 ); /* 0.1 sec timeout */
}

void
switch_state_poll( unsigned char *arg )
{
	toml_table_t* config;
	toml_table_t* hdl_m;
	toml_raw_t raw;
	long long int man_handle_status;
	long long int man_handle_switch;
	unsigned char switch_poll_timer_handle;
	unsigned int handle_state;
	unsigned int hot_swap_sw;

	FILE* fp;
	char errbuf[1000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r");
		logger("ERROR", "fopen() in switch_state_poll() (CONFIG.toml)");

	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in switch_state_poll() (CONFIG.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [HANDLE_SWITCH_M] table. */
	if (0 == (hdl_m = toml_table_in(config, "HANDLE_SWITCH_M"))) {
		logger("ERROR", "toml_table_in() 'HANDLE_SWITCH_M' in switch_state_poll()");
	}

	/* Extract 'man_handle_status' config value. */
	if (0 == (raw = toml_raw_in(hdl_m, "man_handle_status"))) {
		logger("ERROR", "toml_raw_in() 'man_handle_status' in switch_state_poll()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &man_handle_status)) {
		logger("ERROR", "toml_rtoi() 'man_handle_status' in switch_state_poll()");
	}

	/* Extract 'man_handle_switch' config value. */
	if (0 == (raw = toml_raw_in(hdl_m, "man_handle_switch"))) {
		logger("ERROR", "toml_raw_in() 'man_handle_switch' in switch_state_poll()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &man_handle_switch)) {
		logger("ERROR", "toml_rtoi() 'man_handle_switch' in switch_state_poll()");
	}

	if (1 == (unsigned char) man_handle_status) {
			handle_state = (unsigned int) man_handle_switch;
	} else {
			hot_swap_sw = reg_read(devmem_ptr, ha_offset);
			handle_state = (hot_swap_sw >> 16)&0x01;
	}

        if (handle_state != hot_swap_handle_last_state) {
                switch (handle_state) {
                        case HANDLE_SWITCH_OPEN:
                                if( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M5_DEACTIVATION_REQUEST ) {
                                        picmg_m6_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M6_DEACTIVATION_IN_PROGRESS ) {
                                        picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M3_ACTIVATION_IN_PROGRESS ) {
                                        picmg_m6_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M2_ACTIVATION_REQUEST ) {
                                        picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M4_ACTIVE ) {
                                        picmg_m5_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M0_NOT_INSTALLED ) {
                                        picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
                                }
                                break;
                        case HANDLE_SWITCH_CLOSED:
                                if( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M2_ACTIVATION_REQUEST ) {
                                        picmg_m3_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M1_INACTIVE ) {
                                        picmg_m2_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M5_DEACTIVATION_REQUEST ) {
                                        picmg_m4_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M3_ACTIVATION_IN_PROGRESS ) {
                                        picmg_m4_state( fru_inventory_cache[0].fru_dev_id );
                                } else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M0_NOT_INSTALLED ) {
                                        picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
                                }
                                break;
		}
	}

	if ( fru[fru_inventory_cache[0].fru_dev_id].locked == 0 && fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M1_INACTIVE )
	{
		picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
		picmg_m2_state( fru_inventory_cache[0].fru_dev_id );
		logger("DEBUG", "M1 -> M2 (locked bit cleared)");
	}

	if ( fru[fru_inventory_cache[0].fru_dev_id].deactivation_locked == 0 && fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M4_ACTIVE )
	{
		picmg_m5_state( fru_inventory_cache[0].fru_dev_id );
		logger("DEBUG", "M4 -> M5 (deactivation_locked bit cleared)");
	}

	if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M6_DEACTIVATION_IN_PROGRESS )
	{
		picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
	}

	hot_swap_handle_last_state = handle_state;

	toml_free(config);

	// Re-start the timer
	timer_add_callout_queue( (void *)&switch_poll_timer_handle,
		       	0.1*SEC, switch_state_poll, 0 ); /* 0.1 sec timeout */
}

void
module_rearm_events( void )
{
                	if( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M5_DEACTIVATION_REQUEST ) {
                        	picmg_m5_state( fru_inventory_cache[0].fru_dev_id );
                	} else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M6_DEACTIVATION_IN_PROGRESS ) {
                                picmg_m6_state( fru_inventory_cache[0].fru_dev_id );
                	} else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M3_ACTIVATION_IN_PROGRESS ) {
                                picmg_m3_state( fru_inventory_cache[0].fru_dev_id );
                	} else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M2_ACTIVATION_REQUEST ) {
                                picmg_m2_state( fru_inventory_cache[0].fru_dev_id );
                	} else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M4_ACTIVE ) {
                                picmg_m4_state( fru_inventory_cache[0].fru_dev_id );
                	} else if ( fru[fru_inventory_cache[0].fru_dev_id].state == FRU_STATE_M1_INACTIVE ) {
                                picmg_m1_state( fru_inventory_cache[0].fru_dev_id );
                	}
}

/*==============================================================
 * MODULE SENSORS
 *==============================================================*/
void
module_sensor_init( void )
{
	unsigned char dev_slave_addr =  module_get_i2c_address( I2C_ADDRESS_LOCAL );

	int file_count = 0;
	int i, n;

	//Common SDR variables
	long long int record_id[2];
	long long int sdr_version;
	long long int record_type;
	long long int record_len;
	long long int rsv1;
	long long int rsv2;
	long long int entity_id;
	long long int oem;

	//Management Controller Device Locator Record (Type 12)
	long long int ch_num;
	long long int acpi_sys_pwr_st_notify_req;
	long long int acpi_dev_pwr_st_notify_req;
	long long int ctrl_logs_init_errs;
	long long int log_init_agent_errs;
	long long int ctrl_init;
	long long int dev_sup_chassis;
	long long int dev_sup_bridge;
	long long int dev_sup_ipmb_evt_gen;
	long long int dev_sup_ipmb_evt_rcv;
	long long int dev_sup_fru_inv;
	long long int dev_sup_sel;
	long long int dev_sup_sdr_rep;
	long long int dev_sup_sensor;
	long long int rsv[3];
	long long int entity_instance;
	long long int dev_id_typ_len;

	//Full Sensor Record (Type 01)
	long long int id_type;
	long long int channel_num;
	long long int sensor_owner_lun;
	long long int sensor_number;
	long long int entity_type;
	long long int entity_instance_num;
	long long int init_scanning;
	long long int init_events;
	long long int init_thresholds;
	long long int init_hysteresis;
	long long int init_sensor_type;
	long long int powerup_evt_generation;
	long long int powerup_sensor_scanning;
	long long int ignore_sensor;
	long long int sensor_manual_support;
	long long int sensor_hysteresis_support;
	long long int sensor_threshold_access;
	long long int event_msg_control;
	long long int sensor_type;
	long long int event_type_code;
	long long int assertion_event_mask;
	long long int deassertion_event_mask;
	long long int reading_mask;
	long long int analog_data_format;
	long long int rate_unit;
	long long int modifier_unit;
	long long int percentage;
	long long int sensor_units2;
	long long int sensor_units3;
	long long int linearization;
	long long int M;
	long long int M_tolerance;
	long long int B;
	long long int B_accuracy;
	long long int accuracy;
	long long int R_exp;
	long long int B_exp;
	long long int analog_characteristic_flags;
	long long int nominal_reading;
	long long int normal_maximum;
	long long int normal_minimum;
	long long int sensor_maximum_reading;
	long long int sensor_minimum_reading;
	long long int upper_non_recoverable_threshold;
	long long int upper_critical_threshold;
	long long int upper_non_critical_threshold;
	long long int lower_non_recoverable_threshold;
	long long int lower_critical_threshold;
	long long int lower_non_critical_threshold;
	long long int positive_going_threshold_hysteresis_value;
	long long int negative_going_threshold_hysteresis_value;
	long long int reserved2;
	long long int reserved3;
	long long int id_str_typ_len;

	DIR * dirp;
	struct dirent * entry;

	dirp = opendir("/root/UF_IPMC/SDR");
	while ((entry = readdir(dirp)) != NULL) {
    		if (entry->d_type == DT_REG) { /* If the entry is a regular file */
         		file_count++;
    		}
	}
	closedir(dirp);

	for (i=0; i<file_count; i++)
	{
		FILE* fp;
		toml_table_t* conf;
		char errbuf[2000];
		char filename[20];
		char path[100];

		toml_table_t* SDR;
		toml_array_t* arr;
		toml_raw_t raw;

		snprintf(filename, 19, "SDR%d", i);
        	snprintf(path, 99, "/root/UF_IPMC/SDR/SDR%d.toml", i);

		/* Open the file. */
		if (0 == (fp = fopen(path, "r"))) {
			fp = fopen(path, "r");
			logger("ERROR", "fopen() in module_sensor_init() (SDR*.toml)");
		}
		/* Run the file through the parser. */
		conf = toml_parse_file(fp, errbuf, sizeof(errbuf));
		if (0 == conf) {
			logger("ERROR", "toml_parse_file() in module_sensor_init() (SDR*.toml)");
			if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
				exit(EXIT_FAILURE);
			}
		}

		if (fclose(fp) < 0) {
			fclose(fp);
			perror("fclose() failed");
		}

		/* Locate the [SDR*] table. */
		if (0 == (SDR = toml_table_in(conf, filename))) {
			logger("ERROR", "toml_table_in() 'SDR*' in module_sensor_init()");
		}

		if (0 == i) {

			/* Extract 'record_id' config array. */
			if (0 == (arr = toml_array_in(SDR, "record_id"))) {
				logger("ERROR", "toml_array_in() 'record_id' in module_sensor_init()");
			}

			/* Extract 'record_id[0]' config value. */
			if (0 == (raw = toml_raw_at(arr, 0))) {
				logger("ERROR", "toml_raw_at() 'record_id[0]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_id[0])) {
				logger("ERROR", "toml_rtoi() 'record_id[0]' in module_sensor_init()");
			}

			/* Extract 'record_id[1]' config value. */
			if (0 == (raw = toml_raw_at(arr, 1))) {
				logger("ERROR", "toml_raw_at() 'record_id[1]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_id[1])) {
				logger("ERROR", "toml_rtoi() 'record_id[1]' in module_sensor_init()");
			}

			/* Extract 'sdr_version' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sdr_version"))) {
				logger("ERROR", "toml_raw_in() 'sdr_version' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sdr_version)) {
				logger("ERROR", "toml_rtoi() 'sdr_version' in module_sensor_init()");
			}

			/* Extract 'record_type' config value. */
			if (0 == (raw = toml_raw_in(SDR, "record_type"))) {
				logger("ERROR", "toml_raw_in() 'record_type' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_type)) {
				logger("ERROR", "toml_rtoi() 'record_type' in module_sensor_init()");
			}

			/* Extract 'record_len' config value. */
			if (0 == (raw = toml_raw_in(SDR, "record_len"))) {
				logger("ERROR", "toml_raw_in() 'record_len' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_len)) {
				logger("ERROR", "toml_rtoi() 'record_len' in module_sensor_init()");
			}

			/* Extract 'ch_num' config value. */
			if (0 == (raw = toml_raw_in(SDR, "ch_num"))) {
				logger("ERROR", "toml_raw_in() 'ch_num' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &ch_num)) {
				logger("ERROR", "toml_rtoi() 'ch_num' in module_sensor_init()");
			}

			/* Extract 'acpi_sys_pwr_st_notify_req' config value. */
			if (0 == (raw = toml_raw_in(SDR, "acpi_sys_pwr_st_notify_req"))) {
				logger("ERROR", "toml_raw_in() 'acpi_sys_pwr_st_notify_req' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &acpi_sys_pwr_st_notify_req)) {
				logger("ERROR", "toml_rtoi() 'acpi_sys_pwr_st_notify_req' in module_sensor_init()");
			}

			/* Extract 'acpi_dev_pwr_st_notify_req' config value. */
			if (0 == (raw = toml_raw_in(SDR, "acpi_dev_pwr_st_notify_req"))) {
				logger("ERROR", "toml_raw_in() 'acpi_dev_pwr_st_notify_req' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &acpi_dev_pwr_st_notify_req)) {
				logger("ERROR", "toml_rtoi() 'acpi_dev_pwr_st_notify_req' in module_sensor_init()");
			}

			/* Extract 'rsv1' config value. */
			if (0 == (raw = toml_raw_in(SDR, "rsv1"))) {
				logger("ERROR", "toml_raw_in() 'rsv1' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv1)) {
				logger("ERROR", "toml_rtoi() 'rsv1' in module_sensor_init()");
			}

			/* Extract 'rsv2' config value. */
			if (0 == (raw = toml_raw_in(SDR, "rsv2"))) {
				logger("ERROR", "toml_raw_in() 'rsv2' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv2)) {
				logger("ERROR", "toml_rtoi() 'rsv2' in module_sensor_init()");
			}

			/* Extract 'ctrl_logs_init_errs' config value. */
			if (0 == (raw = toml_raw_in(SDR, "ctrl_logs_init_errs"))) {
				logger("ERROR", "toml_raw_in() 'ctrl_logs_init_errs' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &ctrl_logs_init_errs)) {
				logger("ERROR", "toml_rtoi() 'ctrl_logs_init_errs' in module_sensor_init()");
			}

			/* Extract 'log_init_agent_errs' config value. */
			if (0 == (raw = toml_raw_in(SDR, "log_init_agent_errs"))) {
				logger("ERROR", "toml_raw_in() 'log_init_agent_errs' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &log_init_agent_errs)) {
				logger("ERROR", "toml_rtoi() 'log_init_agent_errs' in module_sensor_init()");
			}

			/* Extract 'ctrl_init' config value. */
			if (0 == (raw = toml_raw_in(SDR, "ctrl_init"))) {
				logger("ERROR", "toml_raw_in() 'ctrl_init' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &ctrl_init)) {
				logger("ERROR", "toml_rtoi() 'ctrl_init' in module_sensor_init()");
			}

			/* Extract 'dev_sup_chassis' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_chassis"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_chassis' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_chassis)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_chassis' in module_sensor_init()");
			}

			/* Extract 'dev_sup_bridge' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_bridge"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_bridge' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_bridge)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_bridge' in module_sensor_init()");
			}

			/* Extract 'dev_sup_ipmb_evt_gen' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_ipmb_evt_gen"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_ipmb_evt_gen' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_ipmb_evt_gen)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_ipmb_evt_gen' in module_sensor_init()");
			}

			/* Extract 'dev_sup_ipmb_evt_rcv' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_ipmb_evt_rcv"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_ipmb_evt_rcv' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_ipmb_evt_rcv)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_ipmb_evt_rcv' in module_sensor_init()");
			}

			/* Extract 'dev_sup_fru_inv' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_fru_inv"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_fru_inv' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_fru_inv)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_fru_inv' in module_sensor_init()");
			}

			/* Extract 'dev_sup_sel' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_sel"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_sel' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_sel)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_sel' in module_sensor_init()");
			}

			/* Extract 'dev_sup_sdr_rep' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_sdr_rep"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_sdr_rep' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_sdr_rep)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_sdr_rep' in module_sensor_init()");
			}

			/* Extract 'dev_sup_sensor' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_sup_sensor"))) {
				logger("ERROR", "toml_raw_in() 'dev_sup_sensor' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_sup_sensor)) {
				logger("ERROR", "toml_rtoi() 'dev_sup_sensor' in module_sensor_init()");
			}

			/* Extract 'rsv' config array. */
			if (0 == (arr = toml_array_in(SDR, "rsv"))) {
				logger("ERROR", "toml_array_in() 'rsv' in module_sensor_init()");
			}

			/* Extract 'rsv[0]' config value. */
			if (0 == (raw = toml_raw_at(arr, 0))) {
				logger("ERROR", "toml_raw_at() 'rsv[0]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv[0])) {
				logger("ERROR", "toml_rtoi() 'rsv[0]' in module_sensor_init()");
			}

			/* Extract 'rsv[1]' config value. */
			if (0 == (raw = toml_raw_at(arr, 1))) {
				logger("ERROR", "toml_raw_at() 'rsv[1]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv[1])) {
				logger("ERROR", "toml_rtoi() 'rsv[1]' in module_sensor_init()");
			}

			/* Extract 'rsv[2]' config value. */
			if (0 == (raw = toml_raw_at(arr, 2))) {
				logger("ERROR", "toml_raw_at() 'rsv[2]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv[2])) {
				logger("ERROR", "toml_rtoi() 'rsv[2]' in module_sensor_init()");
			}

			/* Extract 'entity_id' config value. */
			if (0 == (raw = toml_raw_in(SDR, "entity_id"))) {
				logger("ERROR", "toml_raw_in() 'entity_id' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &entity_id)) {
				logger("ERROR", "toml_rtoi() 'entity_id' in module_sensor_init()");
			}

			/* Extract 'entity_instance' config value. */
			if (0 == (raw = toml_raw_in(SDR, "entity_instance"))) {
				logger("ERROR", "toml_raw_in() 'entity_instance' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &entity_instance)) {
				logger("ERROR", "toml_rtoi() 'entity_instance' in module_sensor_init()");
			}

			/* Extract 'oem' config value. */
			if (0 == (raw = toml_raw_in(SDR, "oem"))) {
				logger("ERROR", "toml_raw_in() 'oem' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &oem)) {
				logger("ERROR", "toml_rtoi() 'oem' in module_sensor_init()");
			}

			/* Extract 'dev_id_typ_len' config value. */
			if (0 == (raw = toml_raw_in(SDR, "dev_id_typ_len"))) {
				logger("ERROR", "toml_raw_in() 'dev_id_typ_len' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &dev_id_typ_len)) {
				logger("ERROR", "toml_rtoi() 'dev_id_typ_len' in module_sensor_init()");
			}

			/* Extract 'dev_id_str' config array. */
			if (0 == (arr = toml_array_in(SDR, "dev_id_str"))) {
				logger("ERROR", "toml_array_in() 'dev_id_str' in module_sensor_init()");
			}

			char* dev_id_str[toml_array_nelem(arr)];

			for (n=0; n<toml_array_nelem(arr); n++) {
				/* Extract 'dev_id_str[n]' config value. */
				if (0 == (raw = toml_raw_at(arr, n))) {
					logger("ERROR", "toml_raw_at() 'dev_id_str[n]' in module_sensor_init()");
				};

				/* Convert the raw value into an int. */
				if (toml_rtos(raw, &dev_id_str[n])) {
					logger("ERROR", "toml_rtos() 'dev_id_str[n]' in module_sensor_init()");
				};

				sdr0.dev_id_str[n] = *(char *)dev_id_str[n];
			}

			sdr0.record_id[0] = (unsigned char) record_id[0];					//record_id[0-1] of this record
			sdr0.record_id[1] = (unsigned char) record_id[1];
			sdr0.sdr_version = (unsigned char) sdr_version;						//SDR version
			sdr0.record_type = (unsigned char) record_type;						//Management Controller Locator
			sdr0.record_len = (unsigned char) record_len;						//Number of remaining record bytes following
			sdr0.dev_slave_addr = (unsigned char) dev_slave_addr;					//[7:1] - 7-bit I2C Slave Address of device on channel. Fill during init.
			sdr0.ch_num = (unsigned char) ch_num;							//[3:0] - Channel number for the channel that the management controller is on.

			//BYTE 8
			sdr0.acpi_sys_pwr_st_notify_req = (unsigned char) acpi_sys_pwr_st_notify_req;		//0b = no ACPI System Power State notification required
			sdr0.acpi_dev_pwr_st_notify_req = (unsigned char) acpi_dev_pwr_st_notify_req;		//0b = no ACPI Device Power State notification required
			sdr0.rsv1 = (unsigned char) rsv1;							//reserved
			sdr0.rsv2 = (unsigned char) rsv2;							//reserved
			sdr0.ctrl_logs_init_errs = (unsigned char) ctrl_logs_init_errs;				//1b = Controller logs Initialization
			sdr0.log_init_agent_errs = (unsigned char) log_init_agent_errs;				//1b = Log Initialization Agent errors
			sdr0.ctrl_init = (unsigned char) ctrl_init;						//[1:0] ctrl_init

			//BYTE 9
			sdr0.dev_sup_chassis = (unsigned char) dev_sup_chassis;					//0b = Not a Chassis Device.
			sdr0.dev_sup_bridge = (unsigned char) dev_sup_bridge;					//0b = Not a Bridge
			sdr0.dev_sup_ipmb_evt_gen = (unsigned char) dev_sup_ipmb_evt_gen;			//1b = IPMB Event Generator
			sdr0.dev_sup_ipmb_evt_rcv = (unsigned char) dev_sup_ipmb_evt_rcv;			//1b = IPMB Event Receiver
			sdr0.dev_sup_fru_inv = (unsigned char) dev_sup_fru_inv;					//1b = FRU Inventory Device
			sdr0.dev_sup_sel = (unsigned char) dev_sup_sel;						//0b = Not a SEL Device
			sdr0.dev_sup_sdr_rep = (unsigned char) dev_sup_sdr_rep;					//1b = SDR Repository Device
			sdr0.dev_sup_sensor = (unsigned char) dev_sup_sensor;					//1b = Sensor Device

			sdr0.rsv[0] = (unsigned char) rsv[0];								//rsv[3] - reserved
			sdr0.rsv[1] = (unsigned char) rsv[1];
			sdr0.rsv[2] = (unsigned char) rsv[2];
			sdr0.entity_id = (unsigned char) entity_id;						//entity_id
			sdr0.entity_instance = (unsigned char) entity_instance;					//entity_instance
			sdr0.oem = (unsigned char) oem;								//Reserved for OEM use
			sdr0.dev_id_typ_len = (unsigned char) dev_id_typ_len;					//ASCII | 7 bytes

			sdr_entry_table[0].record_ptr = (unsigned char *)&sdr0;
			sdr_entry_table[0].rec_len = 23;
			sdr_entry_table[0].record_id = current_sensor_count;

			mgmt_add(&sdr0);
		} else {

			/* Extract 'record_id' config array. */
			if (0 == (arr = toml_array_in(SDR, "record_id"))) {
				logger("ERROR", "toml_array_in() 'record_id' in module_sensor_init()");
			}

			/* Extract 'record_id[0]' config value. */
			if (0 == (raw = toml_raw_at(arr, 0))) {
				logger("ERROR", "toml_raw_at() 'record_id[0]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_id[0])) {
				logger("ERROR", "toml_rtoi() 'record_id[0]' in module_sensor_init()");
			}

			/* Extract 'record_id[1]' config value. */
			if (0 == (raw = toml_raw_at(arr, 1))) {
				logger("ERROR", "toml_raw_at() 'record_id[1]' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_id[1])) {
				logger("ERROR", "toml_rtoi() 'record_id[1]' in module_sensor_init()");

			}

			/* Extract 'sdr_version' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sdr_version"))) {
				logger("ERROR", "toml_raw_in() 'sdr_version' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sdr_version)) {
				logger("ERROR", "toml_rtoi() 'sdr_version' in module_sensor_init()");
			}

			/* Extract 'record_type' config value. */
			if (0 == (raw = toml_raw_in(SDR, "record_type"))) {
				logger("ERROR", "toml_raw_in() 'record_type' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_type)) {
				logger("ERROR", "toml_rtoi() 'record_type' in module_sensor_init()");
			}

			/* Extract 'record_len' config value. */
			if (0 == (raw = toml_raw_in(SDR, "record_len"))) {
				logger("ERROR", "toml_raw_in() 'record_len' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &record_len)) {
				logger("ERROR", "toml_rtoi() 'record_len' in module_sensor_init()");
			}

			/* Extract 'id_type' config value. */
			if (0 == (raw = toml_raw_in(SDR, "id_type"))) {
				logger("ERROR", "toml_raw_in() 'id_type' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &id_type)) {
				logger("ERROR", "toml_rtoi() 'id_type' in module_sensor_init()");
			}

			/* Extract 'channel_num' config value. */
			if (0 == (raw = toml_raw_in(SDR, "channel_num"))) {
				logger("ERROR", "toml_raw_in() 'channel_num' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &channel_num)) {
				logger("ERROR", "toml_rtoi() 'channel_num' in module_sensor_init()");
			}

			/* Extract 'rsv1' config value. */
			if (0 == (raw = toml_raw_in(SDR, "rsv1"))) {
				logger("ERROR", "toml_raw_in() 'rsv1' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv1)) {
				logger("ERROR", "toml_rtoi() 'rsv1' in module_sensor_init()");
			}

			/* Extract 'sensor_owner_lun' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_owner_lun"))) {
				logger("ERROR", "toml_raw_in() 'sensor_owner_lun' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_owner_lun)) {
				logger("ERROR", "toml_rtoi() 'sensor_owner_lun' in module_sensor_init()");
			}

			/* Extract 'sensor number' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_number"))) {
				logger("ERROR", "toml_raw_in() 'sensor_number' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_number)) {
				logger("ERROR", "toml_rtoi() 'sensor_number' in module_sensor_init()");
			}

			/* Extract 'entity_id' config value. */
			if (0 == (raw = toml_raw_in(SDR, "entity_id"))) {
				logger("ERROR", "toml_raw_in() 'entity_id' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &entity_id)) {
				logger("ERROR", "toml_rtoi() 'entity_id' in module_sensor_init()");
			}

			/* Extract 'entity_type' config value. */
			if (0 == (raw = toml_raw_in(SDR, "entity_type"))) {
				logger("ERROR", "toml_raw_in() 'entity_type' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &entity_type)) {
				logger("ERROR", "toml_rtoi() 'entity_type' in module_sensor_init()");
			}

			/* Extract 'entity_instance_num' config value. */
			if (0 == (raw = toml_raw_in(SDR, "entity_instance_num"))) {
				logger("ERROR", "toml_raw_in() 'entity_instance_num' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &entity_instance_num)) {
				logger("ERROR", "toml_rtoi() 'entity_instance_num' in module_sensor_init()");
			}

			/* Extract 'rsv2' config value. */
			if (0 == (raw = toml_raw_in(SDR, "rsv2"))) {
				logger("ERROR", "toml_raw_in() 'rsv2' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rsv2)) {
				logger("ERROR", "toml_rtoi() 'rsv2' in module_sensor_init()");
			}

			/* Extract 'init_scanning' config value. */
			if (0 == (raw = toml_raw_in(SDR, "init_scanning"))) {
				logger("ERROR", "toml_raw_in() 'init_scanning' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &init_scanning)) {
				logger("ERROR", "toml_rtoi() 'init_scanning' in module_sensor_init()");
			}

			/* Extract 'init_events' config value. */
			if (0 == (raw = toml_raw_in(SDR, "init_events"))) {
				logger("ERROR", "toml_raw_in() 'init_events' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &init_events)) {
				logger("ERROR", "toml_rtoi() 'init_events' in module_sensor_init()");
			}

			/* Extract 'init_thresholds' config value. */
			if (0 == (raw = toml_raw_in(SDR, "init_thresholds"))) {
				logger("ERROR", "toml_raw_in() 'init_thresholds' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &init_thresholds)) {
				logger("ERROR", "toml_rtoi() 'init_thresholds' in module_sensor_init()");
			}

			/* Extract 'init_hysteresis' config value. */
			if (0 == (raw = toml_raw_in(SDR, "init_hysteresis"))) {
				logger("ERROR", "toml_raw_in() 'init_hysteresis' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &init_hysteresis)) {
				logger("ERROR", "toml_rtoi() 'init_hysteresis' in module_sensor_init()");
			}

			/* Extract 'init_sensor_type' config value. */
			if (0 == (raw = toml_raw_in(SDR, "init_sensor_type"))) {
				logger("ERROR", "toml_raw_in() 'init_sensor_type' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &init_sensor_type)) {
				logger("ERROR", "toml_rtoi() 'init_sensor_type' in module_sensor_init()");
			}

			/* Extract 'powerup_evt_generation' config value. */
			if (0 == (raw = toml_raw_in(SDR, "powerup_evt_generation"))) {
				logger("ERROR", "toml_raw_in() 'powerup_evt_generation' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &powerup_evt_generation)) {
				logger("ERROR", "toml_rtoi() 'powerup_evt_generation' in module_sensor_init()");
			}

			/* Extract 'powerup_sensor_scanning' config value. */
			if (0 == (raw = toml_raw_in(SDR, "powerup_sensor_scanning"))) {
				logger("ERROR", "toml_raw_in() 'powerup_sensor_scanning' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &powerup_sensor_scanning)) {
				logger("ERROR", "toml_rtoi() 'powerup_sensor_scanning' in module_sensor_init()");
			}

			/* Extract 'ignore_sensor' config value. */
			if (0 == (raw = toml_raw_in(SDR, "ignore_sensor"))) {
				logger("ERROR", "toml_raw_in() 'ignore_sensor' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &ignore_sensor)) {
				logger("ERROR", "toml_rtoi() 'ignore_sensor' in module_sensor_init()");
			}

			/* Extract 'sensor_manual_support' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_manual_support"))) {
				logger("ERROR", "toml_raw_in() 'sensor_manual_support' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_manual_support)) {
				logger("ERROR", "toml_rtoi() 'sensor_manual_support' in module_sensor_init()");
			}

			/* Extract 'sensor_hysteresis_support' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_hysteresis_support"))) {
				logger("ERROR", "toml_raw_in() 'sensor_hysteresis_support' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_hysteresis_support)) {
				logger("ERROR", "toml_rtoi() 'sensor_hysteresis_support' in module_sensor_init()");
			}

			/* Extract 'sensor_threshold_access' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_threshold_access"))) {
				logger("ERROR", "toml_raw_in() 'sensor_threshold_access' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_threshold_access)) {
				logger("ERROR", "toml_rtoi() 'sensor_threshold_access' in module_sensor_init()");
			}

			/* Extract 'event_msg_control' config value. */
			if (0 == (raw = toml_raw_in(SDR, "event_msg_control"))) {
				logger("ERROR", "toml_raw_in() 'event_msg_control' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &event_msg_control)) {
				logger("ERROR", "toml_rtoi() 'event_msg_control' in module_sensor_init()");
			}

			/* Extract 'sensor_type' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_type"))) {
				logger("ERROR", "toml_raw_in() 'sensor_type' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_type)) {
				logger("ERROR", "toml_rtoi() 'sensor_type' in module_sensor_init()");
			}

			/* Extract 'event_type_code' config value. */
			if (0 == (raw = toml_raw_in(SDR, "event_type_code"))) {
				logger("ERROR", "toml_raw_in() 'event_type_code' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &event_type_code)) {
				logger("ERROR", "toml_rtoi() 'event_type_code' in module_sensor_init()");
			}

			/* Extract 'assertion_event_mask' config value. */
			if (0 == (raw = toml_raw_in(SDR, "assertion_event_mask"))) {
				logger("ERROR", "toml_raw_in() 'assertion_event_mask' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &assertion_event_mask)) {
				logger("ERROR", "toml_rtoi() 'assertion_event_mask' in module_sensor_init()");
			}

			/* Extract 'deassertion_event_mask' config value. */
			if (0 == (raw = toml_raw_in(SDR, "deassertion_event_mask"))) {
				logger("ERROR", "toml_raw_in() 'deassertion_event_mask' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &deassertion_event_mask)) {
				logger("ERROR", "toml_rtoi() 'deassertion_event_mask' in module_sensor_init()");
			}

			/* Extract 'reading_mask' config value. */
			if (0 == (raw = toml_raw_in(SDR, "reading_mask"))) {
				logger("ERROR", "toml_raw_in() 'reading_mask' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &reading_mask)) {
				logger("ERROR", "toml_rtoi() 'reading_mask' in module_sensor_init()");
			}

			/* Extract 'analog_data_format' config value. */
			if (0 == (raw = toml_raw_in(SDR, "analog_data_format"))) {
				logger("ERROR", "toml_raw_in() 'analog_data_format' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &analog_data_format)) {
				logger("ERROR", "toml_rtoi() 'analog_data_format' in module_sensor_init()");
			}

			/* Extract 'rate_unit' config value. */
			if (0 == (raw = toml_raw_in(SDR, "rate_unit"))) {
				logger("ERROR", "toml_raw_in() 'rate_unit' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &rate_unit)) {
				logger("ERROR", "toml_rtoi() 'rate_unit' in module_sensor_init()");
			}

			/* Extract 'modifier_unit' config value. */
			if (0 == (raw = toml_raw_in(SDR, "modifier_unit"))) {
				logger("ERROR", "toml_raw_in() 'modifier_unit' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &modifier_unit)) {
				logger("ERROR", "toml_rtoi() 'modifier_unit' in module_sensor_init()");
			}

			/* Extract 'percentage' config value. */
			if (0 == (raw = toml_raw_in(SDR, "percentage"))) {
				logger("ERROR", "toml_raw_in() 'percentage' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &percentage)) {
				logger("ERROR", "toml_rtoi() 'percentage' in module_sensor_init()");
			}

			/* Extract 'sensor_units2' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_units2"))) {
				logger("ERROR", "toml_raw_in() 'sensor_units2' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_units2)) {
				logger("ERROR", "toml_rtoi() 'sensor_units2' in module_sensor_init()");
			}

			/* Extract 'sensor_units3' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_units3"))) {
				logger("ERROR", "toml_raw_in() 'sensor_units3' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_units3)) {
				logger("ERROR", "toml_rtoi() 'sensor_units3' in module_sensor_init()");
			}

			/* Extract 'linearization' config value. */
			if (0 == (raw = toml_raw_in(SDR, "linearization"))) {
				logger("ERROR", "toml_raw_in() 'linearization' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &linearization)) {
				logger("ERROR", "toml_rtoi() 'linearization' in module_sensor_init()");
			}

			/* Extract 'M' config value. */
			if (0 == (raw = toml_raw_in(SDR, "M"))) {
				logger("ERROR", "toml_raw_in() 'M' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &M)) {
				logger("ERROR", "toml_rtoi() 'M' in module_sensor_init()");
			}

			/* Extract 'M_tolerance' config value. */
			if (0 == (raw = toml_raw_in(SDR, "M_tolerance"))) {
				logger("ERROR", "toml_raw_in() 'M_tolerance' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &M_tolerance)) {
				logger("ERROR", "toml_rtoi() 'M_tolerance' in module_sensor_init()");
			}

			/* Extract 'B' config value. */
			if (0 == (raw = toml_raw_in(SDR, "B"))) {
				logger("ERROR", "toml_raw_in() 'B' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &B)) {
				logger("ERROR", "toml_rtoi() 'B' in module_sensor_init()");
			}

			/* Extract 'B_accuracy' config value. */
			if (0 == (raw = toml_raw_in(SDR, "B_accuracy"))) {
				logger("ERROR", "toml_raw_in() 'B_accuracy' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &B_accuracy)) {
				logger("ERROR", "toml_rtoi() 'B_accuracy' in module_sensor_init()");
			}

			/* Extract 'accuracy' config value. */
			if (0 == (raw = toml_raw_in(SDR, "accuracy"))) {
				logger("ERROR", "toml_raw_in() 'accuracy' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &accuracy)) {
				logger("ERROR", "toml_rtoi() 'accuracy' in module_sensor_init()");
			}

			/* Extract 'K1' config value. */
			if (0 == (raw = toml_raw_in(SDR, "K1"))) {
				logger("ERROR", "toml_raw_in() 'K1' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &R_exp)) {
				logger("ERROR", "toml_rtoi() 'K1' in module_sensor_init()");
			}

			/* Extract 'K2' config value. */
			if (0 == (raw = toml_raw_in(SDR, "K2"))) {
				logger("ERROR", "toml_raw_in() 'K2' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &B_exp)) {
				logger("ERROR", "toml_rtoi() 'K2' in module_sensor_init()");
			}

			/* Extract 'analog_characteristic_flags' config value. */
			if (0 == (raw = toml_raw_in(SDR, "analog_characteristic_flags"))) {
				logger("ERROR", "toml_raw_in() 'analog_characteristic_flags' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &analog_characteristic_flags)) {
				logger("ERROR", "toml_rtoi() 'analog_characteristic_flags' in module_sensor_init()");
			}

			/* Extract 'nominal_reading' config value. */
			if (0 == (raw = toml_raw_in(SDR, "nominal_reading"))) {
				logger("ERROR", "toml_raw_in() 'nominal_reading' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &nominal_reading)) {
				logger("ERROR", "toml_rtoi() 'nominal_reading' in module_sensor_init()");
			}

			/* Extract 'normal_maximum' config value. */
			if (0 == (raw = toml_raw_in(SDR, "normal_maximum"))) {
				logger("ERROR", "toml_raw_in() 'normal_maximum' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &normal_maximum)) {
				logger("ERROR", "toml_rtoi() 'normal_maximum' in module_sensor_init()");
			}

			/* Extract 'normal_minimum' config value. */
			if (0 == (raw = toml_raw_in(SDR, "normal_minimum"))) {
				logger("ERROR", "toml_raw_in() 'normal_minimum' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &normal_minimum)) {
				logger("ERROR", "toml_rtoi() 'normal_minimum' in module_sensor_init()");
			}

			/* Extract 'sensor_maximum_reading' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_maximum_reading"))) {
				logger("ERROR", "toml_raw_in() 'sensor_maximum_reading' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_maximum_reading)) {
				logger("ERROR", "toml_rtoi() 'sensor_maximum_reading' in module_sensor_init()");
			}

			/* Extract 'sensor_minimum_reading' config value. */
			if (0 == (raw = toml_raw_in(SDR, "sensor_minimum_reading"))) {
				logger("ERROR", "toml_raw_in() 'sensor_minimum_reading' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &sensor_minimum_reading)) {
				logger("ERROR", "toml_rtoi() 'sensor_minimum_reading' in module_sensor_init()");
			}

			/* Extract 'upper_non_recoverable_threshold' config value. */
			if (0 == (raw = toml_raw_in(SDR, "upper_non_recoverable_threshold"))) {
				logger("ERROR", "toml_raw_in() 'upper_non_recoverable_threshold' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &upper_non_recoverable_threshold)) {
				logger("ERROR", "toml_rtoi() 'upper_non_recoverable_threshold' in module_sensor_init()");
			}

			/* Extract 'upper_critical_threshold' config value. */
			if (0 == (raw = toml_raw_in(SDR, "upper_critical_threshold"))) {
				logger("ERROR", "toml_raw_in() 'upper_critical_threshold' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &upper_critical_threshold)) {
				logger("ERROR", "toml_rtoi() 'upper_critical_threshold' in module_sensor_init()");
			}

			/* Extract 'upper_non_critical_threshold' config value. */
			if (0 == (raw = toml_raw_in(SDR, "upper_non_critical_threshold"))) {
				logger("ERROR", "toml_raw_in() 'upper_non_critical_threshold' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &upper_non_critical_threshold)) {
				logger("ERROR", "toml_rtoi() 'upper_non_critical_threshold' in module_sensor_init()");
			}

			/* Extract 'lower_non_recoverable_threshold' config value. */
			if (0 == (raw = toml_raw_in(SDR, "lower_non_recoverable_threshold"))) {
				logger("ERROR", "toml_raw_in() 'lower_non_recoverable_threshold' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &lower_non_recoverable_threshold)) {
				logger("ERROR", "toml_rtoi() 'lower_non_recoverable_threshold' in module_sensor_init()");
			}

			/* Extract 'lower_critical_threshold' config value. */
			if (0 == (raw = toml_raw_in(SDR, "lower_critical_threshold"))) {
				logger("ERROR", "toml_raw_in() 'lower_critical_threshold' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &lower_critical_threshold)) {
				logger("ERROR", "toml_rtoi() 'lower_critical_threshold' in module_sensor_init()");
			}

			/* Extract 'lower_non_critical_threshold' config value. */
			if (0 == (raw = toml_raw_in(SDR, "lower_non_critical_threshold"))) {
				logger("ERROR", "toml_raw_in() 'lower_non_critical_threshold' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &lower_non_critical_threshold)) {
				logger("ERROR", "toml_rtoi() 'lower_non_critical_threshold' in module_sensor_init()");
			}

			/* Extract 'positive_going_threshold_hysteresis_value' config value. */
			if (0 == (raw = toml_raw_in(SDR, "positive_going_threshold_hysteresis_value"))) {
				logger("ERROR", "toml_raw_in() 'positive_going_threshold_hysteresis_value' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &positive_going_threshold_hysteresis_value)) {
				logger("ERROR", "toml_rtoi() 'positive_going_threshold_hysteresis_value' in module_sensor_init()");
			}

			/* Extract 'negative_going_threshold_hysteresis_value' config value. */
			if (0 == (raw = toml_raw_in(SDR, "negative_going_threshold_hysteresis_value"))) {
				logger("ERROR", "toml_raw_in() 'negative_going_threshold_hysteresis_value' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &negative_going_threshold_hysteresis_value)) {
				logger("ERROR", "toml_rtoi() 'negative_going_threshold_hysteresis_value' in module_sensor_init()");
			}

			/* Extract 'reserved2' config value. */
			if (0 == (raw = toml_raw_in(SDR, "reserved2"))) {
				logger("ERROR", "toml_raw_in() 'reserved2' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &reserved2)) {
				logger("ERROR", "toml_rtoi() 'reserved2' in module_sensor_init()");
			}

			/* Extract 'reserved3' config value. */
			if (0 == (raw = toml_raw_in(SDR, "reserved3"))) {
				logger("ERROR", "toml_raw_in() 'reserved3' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &reserved3)) {
				logger("ERROR", "toml_rtoi() 'reserved3' in module_sensor_init()");
			}

			/* Extract 'oem' config value. */
			if (0 == (raw = toml_raw_in(SDR, "oem"))) {
				logger("ERROR", "toml_raw_in() 'oem' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &oem)) {
				logger("ERROR", "toml_rtoi() 'oem' in module_sensor_init()");
			}

			/* Extract 'id_str_typ_len' config value. */
			if (0 == (raw = toml_raw_in(SDR, "id_str_typ_len"))) {
				logger("ERROR", "toml_raw_in() 'id_str_typ_len' in module_sensor_init()");
			}

			/* Convert the raw value into an int. */
			if (toml_rtoi(raw, &id_str_typ_len)) {
				logger("ERROR", "toml_rtoi() 'id_str_typ_len' in module_sensor_init()");
			}

			/* Extract 'id_string_bytes' config array. */
			if (0 == (arr = toml_array_in(SDR, "id_string_bytes"))) {
				logger("ERROR", "toml_array_in() 'id_string_bytes' in module_sensor_init()");
			}

			int byte_count = 0;
			char* id_string_bytes[toml_array_nelem(arr)];

			for (n=0; n<toml_array_nelem(arr); n++) {
				/* Extract 'id_string_bytes[n]' config value. */
				if (0 == (raw = toml_raw_at(arr, n))) {
					logger("ERROR", "toml_raw_at() 'id_string_bytes[n]' in module_sensor_init()");
				};

				/* Convert the raw value into an int. */
				if (toml_rtos(raw, &id_string_bytes[n])) {
					logger("ERROR", "toml_rtos() 'id_string_bytes[n]' in module_sensor_init()");
				};

				byte_count++;
				sdr[i].id_string_bytes[n] = *(char *)id_string_bytes[n];
			}

			sdr[i].record_id[0] = record_id[0]; 								//[0-1] of this record
			sdr[i].record_id[1] = record_id[1];
			sdr[i].sdr_version = sdr_version; 								//SDR version
			sdr[i].record_type = record_type;								//Full sensor record
			sdr[i].record_len = record_len;			 						//Number of remaining record bytes following

			//BYTE 6
			sdr[i].owner_id = dev_slave_addr/2;								//7-bit I2C Slave, fill during init
			sdr[i].id_type = id_type;									//0b = owner_id is IPMB Slave Address

			//BYTE 7
			sdr[i].channel_num = channel_num; 								//[7:4] channel_num
			sdr[i].rsv1 = rsv1; 										//[3:2] reserved
			sdr[i].sensor_owner_lun = sensor_owner_lun;							//[1:0] sensor_owner_lun

			sdr[i].sensor_number = sensor_number;								//8 sensor number
			sdr[i].entity_id = entity_id; 									//9 entity_id

			//BYTE 10
			sdr[i].entity_type = entity_type;								//0b = treat entity as a physical entity
			sdr[i].entity_instance_num = entity_instance_num;						//60h-7Fh device-relative Entity Instance.

			//BYTE 11 - Sensor initialization
			sdr[i].rsv2 = rsv2;										//[7] - reserved. Write as 0b
			sdr[i].init_scanning = init_scanning; 								//[6] init_scanning
			sdr[i].init_events = init_events; 								//[5] init_events
			sdr[i].init_thresholds = init_thresholds; 							//[4] - reserved. Write as 0b
			sdr[i].init_hysteresis = init_hysteresis; 							//[3] init_hysteresis
			sdr[i].init_sensor_type = init_sensor_type;							//[2] init_sensor_type
			sdr[i].powerup_evt_generation = powerup_evt_generation; 					//[1] powerup_evt_generation
			sdr[i].powerup_sensor_scanning = powerup_sensor_scanning; 					//[0] powerup_sensor_scanning

			//BYTE 12 - Sensor capabilities
			sdr[i].ignore_sensor = ignore_sensor;								//[7] ignore_sensor
			sdr[i].sensor_manual_support = sensor_manual_support;						//[6] sensor_manual_support
			sdr[i].sensor_hysteresis_support = sensor_hysteresis_support; 					//[5:4] sensor_hysteresis_support
			sdr[i].sensor_threshold_access = sensor_threshold_access;					//[3:2] sensor_threshold_access
			sdr[i].event_msg_control = event_msg_control;							//[1:0] event_msg_control

			sdr[i].sensor_type = sensor_type; 								//13 sensor_type = 1 - Temp sensor
			sdr[i].event_type_code = event_type_code; 							//14 event_type_code

			sdr[i].event_mask = assertion_event_mask; 							//15,16 assertion event_mask
			sdr[i].deassertion_event_mask = deassertion_event_mask;						//17,18 deassertion event mask
			sdr[i].reading_mask = reading_mask; 								//19,20 reading_mask

			//BYTE 21
			sdr[i].analog_data_format = analog_data_format;							//[7:6] Analog (numeric) Data Format - 2s complement (signed)
			sdr[i].rate_unit = rate_unit; 									//[5:3] rate_unit - 000b = none
			sdr[i].modifier_unit = modifier_unit; 								//[2:1] modifier_unit - 00b = none
			sdr[i].percentage = percentage; 								//[0] percentage - 0b

			sdr[i].sensor_units2 = sensor_units2; 								//22 sensor_units2
			sdr[i].sensor_units3 = sensor_units3;								//23 sensor_units3

			sdr[i].linearization = linearization;								//24 linearization
			sdr[i].M = M; 											//25 M
			sdr[i].M_tolerance = M_tolerance;								//26 M Tolerance
			sdr[i].B = B; 											//27 B
			sdr[i].B_accuracy = B_accuracy; 								//28 B Accuracy
			sdr[i].accuracy = accuracy; 									//29 Accuracy, Accuracy exp, Sensor Direction
			sdr[i].R_exp = R_exp;	 									//30 R exp
			sdr[i].B_exp = B_exp;	 									//30 B exp
			sdr[i].analog_characteristic_flags = analog_characteristic_flags;				//31 Analog characteristic flags
			sdr[i].nominal_reading = nominal_reading;							//32 Nominal Reading
			sdr[i].normal_maximum = normal_maximum;								//33 Normal Maximum - Given as a raw value.
			sdr[i].normal_minimum = normal_minimum;								//34 Normal Minimum - Given as a raw value.
			sdr[i].sensor_maximum_reading = sensor_maximum_reading; 					//35 Sensor Maximum Reading
			sdr[i].sensor_minimum_reading = sensor_minimum_reading; 					//36 Sensor Minimum Reading
			sdr[i].upper_non_recoverable_threshold = upper_non_recoverable_threshold; 			//37 Upper non-recoverable Threshold
			sdr[i].upper_critical_threshold = upper_critical_threshold; 					//38 Upper critical Threshold
			sdr[i].upper_non_critical_threshold = upper_non_critical_threshold; 				//39 Upper non-critical Threshold
			sdr[i].lower_non_recoverable_threshold = lower_non_recoverable_threshold; 			//40 Lower non-recoverable Threshold
			sdr[i].lower_critical_threshold = lower_critical_threshold; 					//41 Lower critical Threshold
			sdr[i].lower_non_critical_threshold = lower_non_critical_threshold; 				//42 Lower non-critical Threshold
			sdr[i].positive_going_threshold_hysteresis_value = positive_going_threshold_hysteresis_value; 	//43 Positive-going Threshold Hysteresis value
			sdr[i].negative_going_threshold_hysteresis_value = negative_going_threshold_hysteresis_value; 	//44 Negative-going Threshold Hysteresis value
			sdr[i].reserved2 = reserved2; 									//45 reserved. Write as 00h.
			sdr[i].reserved3 = reserved3; 									//46 reserved. Write as 00h.
			sdr[i].oem = oem; 										//47 OEM - Reserved for OEM use.
			sdr[i].id_str_typ_len = id_str_typ_len;								//48 id_str_typ_len Sensor ID String Type/Length Code, 5 chars in str

			sd[i].sensor_id = (unsigned char) sensor_number;

			sdr_entry_table[current_sensor_count].record_ptr = (unsigned char *)&sdr[i];
			sdr_entry_table[current_sensor_count].rec_len = 48 + byte_count;
			sdr_entry_table[current_sensor_count].record_id = current_sensor_count;

			sensor_add(&sdr[i],&sd[i]);
		}

		toml_free(conf);
	}
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+		FRU Hot Swap Sensor		        	      +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[1].scan_function = read_fru_hot_swap;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+		IPMB-0 Status Sensor  		      	     	      +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[2].scan_function = read_ipmb_0_status;

	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+		Hot Swap Handle Sensor		 	     	      +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[3].scan_function = read_hot_swap_handle;

} //end of module_sensor_init() function

//===============================================================================

void
module_cold_reset( unsigned char dev_id )
{
}

void
module_warm_reset( unsigned char dev_id )
{
}

void
module_graceful_reboot( unsigned char dev_id )
{
}

void
module_issue_diag_int( unsigned char dev_id )
{
}

void
module_quiesce( unsigned char dev_id )
{
}


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+		FRU Hot Swap Sensor		    	  	      +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void
read_fru_hot_swap( void )
{
	sd[1].last_sensor_reading = 0;
	sd[1].std_ipmi_byte = 0xC0;
	sd[1].reserved2 = 0x80;

	switch (fru[fru_inventory_cache[0].fru_dev_id].state) {
		case FRU_STATE_M0_NOT_INSTALLED:
			sd[1].current_state_mask = 0x01;
			break;
                case FRU_STATE_M1_INACTIVE:
                        sd[1].current_state_mask = 0x02;
                        break;
                case FRU_STATE_M2_ACTIVATION_REQUEST:
                        sd[1].current_state_mask = 0x04;
                        break;
                case FRU_STATE_M3_ACTIVATION_IN_PROGRESS:
                        sd[1].current_state_mask = 0x08;
                        break;
                case FRU_STATE_M4_ACTIVE:
                        sd[1].current_state_mask = 0x10;
                        break;
                case FRU_STATE_M5_DEACTIVATION_REQUEST:
                        sd[1].current_state_mask = 0x20;
                        break;
                case FRU_STATE_M6_DEACTIVATION_IN_PROGRESS:
                        sd[1].current_state_mask = 0x40;
                        break;
                case FRU_STATE_M7_COMMUNICATION_LOST:
                        sd[1].current_state_mask = 0x80;
                        break;
	}
}

void
fru_hot_swap_state_poll( unsigned char *arg )
{
	unsigned char fru_hot_swap_timer_handle;

	if ( fru[fru_inventory_cache[0].fru_dev_id].state != FRU_STATE_M4_ACTIVE &&
	     fru[fru_inventory_cache[0].fru_dev_id].state != FRU_STATE_M1_INACTIVE )
	{
		module_rearm_events();
	}

	// Re-start the timer
	timer_add_callout_queue( (void *)&fru_hot_swap_timer_handle,
		       	5*SEC, fru_hot_swap_state_poll, 0 ); /* 5 sec timeout */
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+		IPMB-0 Status Sensor			     	      +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void
read_ipmb_0_status( void )
{
	FRU_IPMB_0_EVENT_MSG_REQ msg;

	unsigned int status_0;
	unsigned int status_1;

	status_0 = (reg_read(devmem_ptr, ha_offset)>>8)&0x01;
	status_1 = (reg_read(devmem_ptr, ha_offset)>>9)&0x01;

	if ( fru_ipmb_a_b_event_set )
	{
		if ( sd[2].ipmb_a_enabled_ipmb_b_disabled == 1 )
		{
			status_0 = 1;
			status_1 = 0;
		}

		if ( sd[2].ipmb_a_disabled_ipmb_b_enabled == 1 )
		{
			status_0 = 0;
			status_1 = 1;
		}

		fru_ipmb_a_b_event_set = 0;
	}

	sd[2].std_ipmi_byte = 0xC0;

	if (status_0)
	{
		sd[2].ipmb_a_local_status = 0;
		sd[2].ipmb_a_override_state = 1;
	} else
	{
		sd[2].ipmb_a_local_status = 0x07;
		sd[2].ipmb_a_override_state = 0;
	}

	if (status_1)
	{
		sd[2].ipmb_b_local_status = 0;
		sd[2].ipmb_b_override_state = 1;
	} else
	{
		sd[2].ipmb_b_local_status = 0x07;
		sd[2].ipmb_b_override_state = 0;
	}

	if (status_0)
	{
		if (status_1)
		{
			sd[2].ipmb_a_enabled_ipmb_b_enabled = 1;
			msg.offset = 0x03;
			msg.evt_direction = 0x6F;

			if (fru_ipmb_a_b_last_state != 1)
			{
				fru_ipmb_a_b_last_state = 1;
				fru_ipmb_a_b_init_state = 1;
			} else
			{
				fru_ipmb_a_b_init_state = 0;
			}
		} else
		{
			sd[2].ipmb_a_enabled_ipmb_b_disabled = 1;
			msg.offset = 0x01;
			msg.evt_direction = 0x6F;

			if (fru_ipmb_a_b_last_state != 2)
			{
				fru_ipmb_a_b_last_state = 2;
				fru_ipmb_a_b_init_state = 1;
			} else
			{
				fru_ipmb_a_b_init_state = 0;
			}
		}
	} else
	{
		if (status_1)
		{
			sd[2].ipmb_a_disabled_ipmb_b_enabled = 1;
			msg.offset = 0x02;
			msg.evt_direction = 0x6F;

			if (fru_ipmb_a_b_last_state != 3)
			{
				fru_ipmb_a_b_last_state = 3;
				fru_ipmb_a_b_init_state = 1;
			} else
			{
				fru_ipmb_a_b_init_state = 0;
			}
		} else
		{
			sd[2].ipmb_a_disabled_ipmb_b_disabled = 1;
			msg.offset = 0x00;
			msg.evt_direction = 0x6F;

			if (fru_ipmb_a_b_last_state != 4)
			{
				fru_ipmb_a_b_last_state = 4;
				fru_ipmb_a_b_init_state = 1;
			} else
			{
				fru_ipmb_a_b_init_state = 0;
			}
		}
	}

	sd[2].reserved2 = 0x80;

	msg.command = 0x02;
	msg.evt_msg_rev = 0x04;
	msg.sensor_type = 0xF1;
	msg.sensor_number = 0x02;
	msg.oem = 0x0A;
	msg.ch_num = 0x00;

	msg.ipmb_a_local_status = sd[2].ipmb_a_local_status;
	msg.ipmb_a_override_state = sd[2].ipmb_a_override_state;
	msg.ipmb_b_local_status = sd[2].ipmb_b_local_status;
	msg.ipmb_b_override_state = sd[2].ipmb_b_override_state;

	if (sd[2].ipmb_a_enabled_ipmb_b_enabled && fru_ipmb_a_b_last_state == 1 && fru_ipmb_a_b_init_state == 1)
	{
		ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_IPMB_0_EVENT_MSG_REQ), 0);
		reg_write(devmem_ptr, qbv_on_off, 0);
	}

	if (sd[2].ipmb_a_enabled_ipmb_b_disabled && fru_ipmb_a_b_last_state == 2 && fru_ipmb_a_b_init_state == 1)
	{
		ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_IPMB_0_EVENT_MSG_REQ), 0);
		reg_write(devmem_ptr, qbv_on_off, 2);
	}

	if (sd[2].ipmb_a_disabled_ipmb_b_enabled && fru_ipmb_a_b_last_state == 3 && fru_ipmb_a_b_init_state == 1)
	{
		ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_IPMB_0_EVENT_MSG_REQ), 0);
		reg_write(devmem_ptr, qbv_on_off, 1);
	}

	if (sd[2].ipmb_a_disabled_ipmb_b_disabled && fru_ipmb_a_b_last_state == 4 && fru_ipmb_a_b_init_state == 1)
	{
		//ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_IPMB_0_EVENT_MSG_REQ), 0);
		reg_write(devmem_ptr, qbv_on_off, 3);
	}

	sd[2].ipmb_a_enabled_ipmb_b_enabled = 0;
	sd[2].ipmb_a_enabled_ipmb_b_disabled = 0;
	sd[2].ipmb_a_disabled_ipmb_b_enabled = 0;
	sd[2].ipmb_a_disabled_ipmb_b_disabled = 0;
}

void
ipmb_0_state_poll( unsigned char *arg )
{
	unsigned char ipmb_0_timer_handle;

	read_ipmb_0_status();

	// Re-start the timer
	timer_add_callout_queue( (void *)&ipmb_0_timer_handle,
		       	3*SEC, ipmb_0_state_poll, 0 ); /* 3 sec timeout */
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+		Hot Swap Handle Sensor		    	  	      +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void
read_hot_swap_handle( void )
{
	sd[3].last_sensor_reading = hot_swap_handle_last_state;
	sd[3].sensor_scanning_enabled = 1;
	sd[3].event_messages_enabled = 1;
	sd[3].unavailable = 0;
	sd[3].current_state_mask = 0;
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+               Watchdog Timer Handler                                +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void
watchdog_ping( void )
{
        if (ioctl(wdt_fd, WDIOC_KEEPALIVE, NULL) < 0) {
                logger("ioctl(WDIOC_KEEPALIVE) in watchdog_ping()", strerror(errno));
        }
}
void
watchdog_state_poll( unsigned char *arg )
{
        unsigned char watchdog_timer_handle;

        watchdog_ping();

        // Re-start the timer
        timer_add_callout_queue( (void *)&watchdog_timer_handle,
                        3*SEC, watchdog_state_poll, 0 ); /* 3 sec timeout */
}
