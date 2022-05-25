/*
   UF_IPMC/user-sensor.c

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
#include "sensor.h"
#include "logger.h"
#include "user-sensor-vu13p-qsfp30.h"
#include "user-payload-vu13p-qsfp30.h"
#include "semaphore.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/limits.h>
#include "octopus.h"

#define qbv_on_off              0x1220000
#define xadc_temp_offset				0x3c30200
#define xadc_cr0_offset        	0x3c30300
#define xadc_cr1_offset        	0x3c30300
#define payload_off_alarm				0x1220008

extern unsigned long long int lbolt;
extern unsigned long long int payload_timeout_init;
extern FRU_CACHE fru_inventory_cache[];
extern unsigned char current_sensor_count;
extern SDR_ENTRY sdr_entry_table[];
extern int i2c_fd_snsr[];
extern FULL_SENSOR_RECORD sdr[];
extern SENSOR_DATA sd[];
extern int optics_powered;

unsigned char temp_max = 0;

// map of the qsfp selection bits.
// qsfp module numbers in comments match schematics
qsfp_select_t qsfp_select [30] =
{
    {0, 2, 7}, // QSFP 0
    {0, 2, 4}, // QSFP 1
    {0, 2, 2}, // QSFP 2
    {0, 1, 7}, // QSFP 3
    {0, 1, 5}, // QSFP 4
    {0, 1, 2}, // QSFP 5
    {1, 0, 0}, // QSFP 6
    {1, 0, 2}, // QSFP 7
    {1, 0, 4}, // QSFP 8
    {1, 0, 6}, // QSFP 9
    {1, 1, 0}, // QSFP 10
    {1, 1, 2}, // QSFP 11
    {1, 1, 4}, // QSFP 12
    {1, 1, 6}, // QSFP 13
    {0, 2, 6}, // QSFP 14
    {0, 2, 3}, // QSFP 15
    {0, 2, 1}, // QSFP 16
    {0, 1, 6}, // QSFP 17
    {0, 1, 4}, // QSFP 18
    {0, 1, 1}, // QSFP 19
    {1, 0, 1}, // QSFP 20
    {1, 0, 3}, // QSFP 21
    {1, 0, 5}, // QSFP 22
    {1, 0, 7}, // QSFP 23
    {1, 1, 1}, // QSFP 24
    {1, 1, 3}, // QSFP 25
    {1, 1, 5}, // QSFP 26
    {1, 1, 7}, // QSFP 27
    {1, 2, 0}, // QSFP 28
    {1, 2, 1}  // QSFP 29
};

/*==============================================================*/
/* Local Function Prototypes					*/
/*==============================================================*/
void pgood_state_poll( unsigned char *arg );
void temp_qsfp30_max_poll( unsigned char *arg );
void temp_DSE0133V2NBC_poll( unsigned char *arg );
void temp_PIM400KZ_poll( unsigned char *arg );
void temp_XADC_poll( unsigned char *arg );

/*==============================================================*/
/* USER SEMAPHORE INITIALIZATION				*/
/*==============================================================*/
void semaphore_initialize(void) {
	if (create_semaphore(3) < 0) {
		logger("ERROR", "Semaphore initialization failed for sensor bus 3");
	} else {
		logger("SUCCESS", "Semaphore initialization complete for sensor bus 3");
	}

	if (create_semaphore(4) < 0) {
		logger("ERROR", "Semaphore initialization failed for sensor bus 4");
	} else {
		logger("SUCCESS", "Semaphore initialization complete for sensor bus 4");
	}
}

void user_sensor_state_poll(void) {
	pgood_state_poll( 0 );
	//temp_qsfp30_max_poll( 0 );
	//temp_DSE0133V2NBC_poll( 0 );
	//temp_PIM400KZ_poll( 0 );
	//temp_XADC_poll( 0 );
} // end of user_module_init() function



/*==============================================================
 *  * USER MODULE SENSORS INITIALIZATION
 *   *==============================================================*/
void user_module_sensor_init(void) {
	/*==============================================================*/
	/* 		All Sensors											*/
	/*==============================================================*/
  sd[4].scan_function = read_sensor_pgood;
	sd[5].scan_function = read_sensor_temp_qsfp30_max;
	sd[6].scan_function = read_sensor_temp_DSE0133V2NBC;
	sd[7].scan_function = read_sensor_temp_PIM400KZ;
	sd[8].scan_function = read_sensor_temp_XADC;
}

/*==============================================================*/
/* 	 		PGOOD Sensor				                        */
/*==============================================================*/
void read_sensor_pgood(void) {
	//	Sensor Data Record
	u8 sensor_N = 4;
	u8 result = 0;
	u8 bot = 0;
	u8 top = 0;
	u8 qsfp = 0;
	if (check_power_up())
	{
		result = reg_read(devmem_ptr, payload_off_alarm);
		bot = (result>>25)&0x1;
		top = (result>>26)&0x1;
		qsfp = (result>>27)&0x1;

		sd[sensor_N].last_sensor_reading = 1;
		sd[sensor_N].sensor_scanning_enabled = 1;
		sd[sensor_N].event_messages_enabled = 1;
		sd[sensor_N].unavailable = 0;

		if (bot || top || qsfp)
		{
			if (bot)
			{
				logger("POK ALARM", "Payload off ALARM in Bottom FPGA");
			}
			if (top)
			{
				logger("POK ALARM", "Payload off ALARM in Top FPGA");
			}
			if (qsfp)
			{
				logger("POK ALARM", "Payload off ALARM in QSFP module");
			}

			picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
		}
	}
	else
	{
		sd[sensor_N].last_sensor_reading = 0;
		sd[sensor_N].sensor_scanning_enabled = 0;
		sd[sensor_N].event_messages_enabled = 0;
		sd[sensor_N].unavailable = 1;
	}
}

/*==============================================================*/
/* 	 		Temp QSFP Max Sensor				                        */
/*==============================================================*/
void read_sensor_temp_qsfp30_max(void) {
    lock(3);

		uint64_t exp0 = 0, exp1 = 0, qsfp_mod = 0;
		exp0 = 0x10; // expansion register on top layer
		exp1 = 0x12; // expansion register on bottom layer
		qsfp_mod = 0x50; // QSFP module address

    //	Sensor Data Record
    u8 sensor_N = 5;
		u8 i2c_ch = 1;

    if (check_power_up()) {
				uint8_t er[2][3]; // expansion register bytes

				for (int i = 0; i < 30; i++)
				{
						//			int i = mod_ind[j]; // temporary
						qsfp_select_t qs = qsfp_select[i];

						// initial values of the exp. regs are all ones
						memset (er, 0xff, sizeof (er));

						// drop selection bit corresponding to selected QSFP
						er[qs.chip_ind][qs.byte_num] &= (~(1 << qs.bit_num));

						// printf ("i: %d reg: %x byte: %d bit: %d %02x%02x%02x %02x%02x%02x\n",
						// i, qs.chip_ind, qs.byte_num, qs.bit_num,
						// er[0][0],er[0][1],er[0][2], er[1][0],er[1][1],er[1][2]);
						// fflush (stdout);

						// write selection bits into expansion registers
						i2c_write_qsfp30 (i2c_fd_snsr[i2c_ch], exp0, er[0]);
						i2c_write_qsfp30 (i2c_fd_snsr[i2c_ch], exp1, er[1]);

						t.t16 = 0;
						int res;
						res = i2c_read (i2c_fd_snsr[i2c_ch], qsfp_mod, 0x16, &t.t8[1]);
						// if the first read fails, the device is not there, skip the rest
						if (res != -1)
						{
								i2c_read (i2c_fd_snsr[i2c_ch], qsfp_mod, 0x17, &t.t8[0]);

								// convert to C according to QSFP-MSA.pdf page 53 line 14
								float temp_f = ((float)t.t16)/256.;

								v.v16 = 0;
								i2c_read (i2c_fd_snsr[i2c_ch], qsfp_mod, 0x1a, &v.v8[1]);
								i2c_read (i2c_fd_snsr[i2c_ch], qsfp_mod, 0x1b, &v.v8[0]);

								// convert to V according to QSFP-MSA.pdf page 53 line 19
								float vv = ((float)v.v16)/10000.;

								if (vv < 3.2)
								{
										printf ("device %d voltage = %1.2f\n", i, vv);
								}

        				//	Convert float to byte and get precision
        				u8 temp_b = (u8)(temp_f);

								if (temp_b >= temp_max)
								{
										temp_max = temp_b;
								}
						}
				}

				sd[sensor_N].last_sensor_reading = temp_max;
				sd[sensor_N].sensor_scanning_enabled = 1;
				sd[sensor_N].event_messages_enabled = 1;
				sd[sensor_N].unavailable = 0;

				temp_max = 0;

        static int first_time = 1;
        static int up_noncrt_assert = 0;

        if (first_time) {
            		first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
						sd[sensor_N].current_state_mask = 0xE0;
            unlock(3);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
          	logger("WARNING","Non-recoverable threshold crossed for QSFP temperature sensor");
            lock(3);
        } else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
						sd[sensor_N].current_state_mask = 0xD0;
						// Assertion message for shelf manager
          	FRU_TEMPERATURE_EVENT_MSG_REQ msg;
          	msg.command = 0x02;
			      msg.evt_msg_rev = 0x04;
          	msg.sensor_type = 0x01;
          	msg.sensor_number = sensor_N;
          	msg.evt_direction = 0x01;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 1;
				} else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_critical_threshold) {
						sd[sensor_N].current_state_mask = 0xC0;
            // Deassertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;
            msg.command = 0x02;
            msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
          	msg.sensor_number = sensor_N;
            msg.evt_direction = 0x81;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 0;
        } else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
						sd[sensor_N].current_state_mask = 0xC8;
            // Assertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;
            msg.command = 0x02;
			      msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x01;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
          	msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 2;
        } else if (up_noncrt_assert == 2 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
						sd[sensor_N].current_state_mask = 0xC0;
          	// Deassertion message for shelf manager
            FRU_TEMPERATURE_EVENT_MSG_REQ msg;
            msg.command = 0x02;
          	msg.evt_msg_rev = 0x04;
            msg.sensor_type = 0x01;
            msg.sensor_number = sensor_N;
            msg.evt_direction = 0x81;
            msg.evt_data2_qual = 0x01;
            msg.evt_data3_qual = 0x01;
            msg.evt_reason = 0x07;
            msg.temp_reading = sd[sensor_N].last_sensor_reading;
            msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

            ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
            up_noncrt_assert = 0;
        }
    } else {
				sd[sensor_N].last_sensor_reading = 0;
				sd[sensor_N].sensor_scanning_enabled = 0;
				sd[sensor_N].event_messages_enabled = 0;
				sd[sensor_N].unavailable = 1;
				sd[sensor_N].current_state_mask = 0;
    }
    unlock(3);
}

/*==============================================================*/
/* 	 		Temp DSE0133V2NBC Sensor				                        */
/*==============================================================*/
void read_sensor_temp_DSE0133V2NBC(void) {
    lock(4);

    //	Sensor Data Record
    u8 sensor_N = 6;
		u8 i2c_ch = 2;
		u8 i2c_addr = 0x36;
		u8 pmbus_cmd = 0x8D;
		u16 result = 0, base = 0, mantissa = 0, exp_sign = 0, base_2s = 0, base_tmp = 0;
    int sign, k = 0;
    double temp;

		pmbus_two_bytes_read(i2c_fd_snsr[i2c_ch], i2c_addr, pmbus_cmd, &result);
    base = result & 0x7ff;
    mantissa = (result >> 11) & 0x1f;
    exp_sign = (mantissa >> 4) & 0x1;
    sign = ((base >> 10) & 0x1) ? -1 : 1;
    k = (exp_sign) ? -(~(mantissa & 0xf) + 1) : ~(mantissa & 0xf) + 1;
    base_2s = ~(base & 0x3ff)+1;
    base_tmp = (sign) ? base : base_2s;
    temp = sign * base_tmp * (double) (1 << k);

		u8 temp_b = (u8)(temp);

		sd[sensor_N].last_sensor_reading = temp_b;
		sd[sensor_N].sensor_scanning_enabled = 1;
		sd[sensor_N].event_messages_enabled = 1;
		sd[sensor_N].unavailable = 0;

		static int first_time = 1;
		static int up_noncrt_assert = 0;

		if (first_time) {
						first_time = 0;
		} else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
				// Transition to M6 for non-recoverable
				sd[sensor_N].current_state_mask = 0xE0;
				unlock(4);
				picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
				logger("WARNING","Non-recoverable threshold crossed for DSE0133V2NBC temperature sensor");
				lock(4);
		} else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xD0;
				// Assertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x01;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 1;
		} else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC0;
				// Deassertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x81;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 0;
		} else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC8;
				// Assertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x01;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 2;
		} else if (up_noncrt_assert == 2 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC0;
				// Deassertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x81;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 0;
		}

    unlock(4);
}

/*==============================================================*/
/* 	 		Temp PIM400KZ Sensor				                        */
/*==============================================================*/
void read_sensor_temp_PIM400KZ(void) {
    lock(4);

    //	Sensor Data Record
    u8 sensor_N = 7;
		u8 i2c_ch = 2;
		u8 i2c_addr = 0x2f;
		u8 pmbus_cmd = 0x28;
		u8 result = 0;

		i2c_read(i2c_fd_snsr[i2c_ch], i2c_addr, pmbus_cmd, &result);
		float temp_f = ((float) result)*1.961 - 50;
		u8 temp_b = (u8)(temp_f);

		sd[sensor_N].last_sensor_reading = temp_b;
		sd[sensor_N].sensor_scanning_enabled = 1;
		sd[sensor_N].event_messages_enabled = 1;
		sd[sensor_N].unavailable = 0;

		static int first_time = 1;
		static int up_noncrt_assert = 0;

		if (first_time) {
						first_time = 0;
		} else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
				// Transition to M6 for non-recoverable
				sd[sensor_N].current_state_mask = 0xE0;
				unlock(4);
				picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
				logger("WARNING","Non-recoverable threshold crossed for PIM400KZ temperature sensor");
				lock(4);
		} else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xD0;
				// Assertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x01;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 1;
		} else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC0;
				// Deassertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x81;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 0;
		} else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC8;
				// Assertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x01;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 2;
		} else if (up_noncrt_assert == 2 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC0;
				// Deassertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x81;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 0;
		}

    unlock(4);
}

/*==============================================================*/
/* 	 		Temp XADC Sensor				                        */
/*==============================================================*/
void read_sensor_temp_XADC(void) {
    //	Sensor Data Record
    u8 sensor_N = 8;
		u32 result = 0;
    u16 temp_d = 0;
    u32 xadc_reg_rw;

    xadc_reg_rw = reg_read(devmem_ptr, xadc_cr0_offset);
    xadc_reg_rw &= ~0x40ff;
    reg_write(devmem_ptr, xadc_cr0_offset, xadc_reg_rw);

    xadc_reg_rw = reg_read(devmem_ptr, xadc_cr1_offset);
    xadc_reg_rw &= ~0xf000;
    reg_write(devmem_ptr, xadc_cr1_offset, xadc_reg_rw);

		result = reg_read(devmem_ptr, xadc_temp_offset);
    temp_d = (result >> 4)&0xfff;
		float temp_f = (((float) temp_d)*503.975)/4096. - 273.15;
		u8 temp_b = (u8)(temp_f);

		sd[sensor_N].last_sensor_reading = temp_b;
		sd[sensor_N].sensor_scanning_enabled = 1;
		sd[sensor_N].event_messages_enabled = 1;
		sd[sensor_N].unavailable = 0;

		static int first_time = 1;
		static int up_noncrt_assert = 0;

		if (first_time) {
						first_time = 0;
		} else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
				// Transition to M6 for non-recoverable
				sd[sensor_N].current_state_mask = 0xE0;
				picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
				logger("WARNING","Non-recoverable threshold crossed for XADC temperature sensor");
		} else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xD0;
				// Assertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x01;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 1;
		} else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC0;
				// Deassertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x81;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 0;
		} else if (up_noncrt_assert == 0 && sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC8;
				// Assertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x01;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 2;
		} else if (up_noncrt_assert == 2 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
				sd[sensor_N].current_state_mask = 0xC0;
				// Deassertion message for shelf manager
				FRU_TEMPERATURE_EVENT_MSG_REQ msg;
				msg.command = 0x02;
				msg.evt_msg_rev = 0x04;
				msg.sensor_type = 0x01;
				msg.sensor_number = sensor_N;
				msg.evt_direction = 0x81;
				msg.evt_data2_qual = 0x01;
				msg.evt_data3_qual = 0x01;
				msg.evt_reason = 0x07;
				msg.temp_reading = sd[sensor_N].last_sensor_reading;
				msg.threshold = sdr[sensor_N].upper_non_critical_threshold;

				ipmi_send_event_req(( unsigned char * )&msg, sizeof(FRU_TEMPERATURE_EVENT_MSG_REQ), 0);
				up_noncrt_assert = 0;
		}
}

void pgood_state_poll( unsigned char *arg ) {
        unsigned char pgood_timer_handle;

        read_sensor_pgood();

        // Re-start the timer
        timer_add_callout_queue( (void *)&pgood_timer_handle,
                        0.1*SEC, pgood_state_poll, 0 ); /* 0.1 sec timeout */
}

void temp_qsfp30_max_poll( unsigned char *arg ) {
        unsigned char temp_qsfp30_max_timer_handle;

        read_sensor_temp_qsfp30_max();

        // Re-start the timer
        timer_add_callout_queue( (void *)&temp_qsfp30_max_timer_handle,
                        5*SEC, temp_qsfp30_max_poll, 0 ); /* 5 sec timeout */
}

void temp_DSE0133V2NBC_poll( unsigned char *arg ) {
        unsigned char temp_DSE0133V2NBC_timer_handle;

        read_sensor_temp_DSE0133V2NBC();

        // Re-start the timer
        timer_add_callout_queue( (void *)&temp_DSE0133V2NBC_timer_handle,
                        5*SEC, temp_DSE0133V2NBC_poll, 0 ); /* 5 sec timeout */
}

void temp_PIM400KZ_poll( unsigned char *arg ) {
        unsigned char temp_PIM400KZ_timer_handle;

        read_sensor_temp_DSE0133V2NBC();

        // Re-start the timer
        timer_add_callout_queue( (void *)&temp_PIM400KZ_timer_handle,
                        5*SEC, temp_PIM400KZ_poll, 0 ); /* 5 sec timeout */
}

void temp_XADC_poll( unsigned char *arg ) {
        unsigned char temp_XADC_timer_handle;

        read_sensor_temp_XADC();

        // Re-start the timer
        timer_add_callout_queue( (void *)&temp_XADC_timer_handle,
                        5*SEC, temp_XADC_poll, 0 ); /* 5 sec timeout */
}
