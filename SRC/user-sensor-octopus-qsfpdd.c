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
#include "user-sensor-octopus-qsfpdd.h"
#include "user-payload-octopus-qsfpdd.h"
#include "semaphore.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include "octopus.h"

#define qbv_on_off              0x1220000

extern unsigned long long int lbolt;
extern unsigned long long int payload_timeout_init;
extern FRU_CACHE fru_inventory_cache[];
extern unsigned char current_sensor_count;
extern SDR_ENTRY sdr_entry_table[];
extern int i2c_fd_snsr[];
extern FULL_SENSOR_RECORD sdr[];
extern SENSOR_DATA sd[];
extern int optics_powered;

/*==============================================================*/
/* USER SEMAPHORE INITIALIZATION				*/
/*==============================================================*/
void semaphore_initialize(void) {
	if (create_semaphore(2) < 0) {
		logger("ERROR", "Semaphore initialization failed for sensor bus 2");
	} else {
		logger("SUCCESS", "Semaphore initialization complete for sensor bus 2");
    }

	if (create_semaphore(3) < 0) {
		logger("ERROR", "Semaphore initialization failed for sensor bus 3");
	} else {
		logger("SUCCESS", "Semaphore initialization complete for sensor bus 3");
	}
}

void user_sensor_state_poll(void) {
} // end of user_module_init() function



/*==============================================================
 *  * USER MODULE SENSORS INITIALIZATION
 *   *==============================================================*/
void user_module_sensor_init(void) {
	/*==============================================================*/
	/* 		All Sensors											*/
	/*==============================================================*/
  sd[4].scan_function = read_all_i2c_sensors;
  sd[5].scan_function = read_sensor_temp_vup_remote;
  sd[6].scan_function = read_sensor_temp_k7_remote;
  sd[7].scan_function = read_sensor_temp_rail_2v7_intermediate_remote;
  sd[8].scan_function = read_sensor_temp_mgt_vup_remote;
  sd[9].scan_function = read_sensor_optics;
}



/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * +	OCTOPUS and X20 PGOOD Sensor					      +
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void read_all_i2c_sensors(void) {
  //Check that monitoring is done
  //if not wait for next poll
  u8 ready=0;
  u8 pgood=0;
  lock(2);
  while(!ready) {
    i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x03, &ready);
    ready=((ready &0x40)==0);
  }
  if (ready & check_power_up()) {
    //read PGOOD
    i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 0x04, &pgood);
    sd[4].last_sensor_reading = pgood;
    sd[4].sensor_scanning_enabled = 1;
    sd[4].event_messages_enabled = 1;
    sd[4].unavailable = 0;
    sd[4].current_state_mask = 0;
    //PGOOD shutdown
    if (sd[4].last_sensor_reading != 0xff) {
      logger("WARNING","PGOOD sensor reading too low");
      printf("PGOOD sensor reading: %d\n",sd[4].last_sensor_reading);
      unlock(2);
      picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
      lock(2);
    }
    //read all temperatures
    float temp_f=30;
    u8 i=0;
    u8 msb=0;
    u8 lsb=0;
    u8 tb = 0;
    for (i=12;i<56;i=i+4) {
      i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 7+i, &lsb);
      i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 7+i+1, &msb);
      int tempint = (lsb>>4)|(msb<<4);
      float t = twos_complement(tempint,12)*0.0625;
      tb = t;
      if (i==44) {
	sd[5].last_sensor_reading = tb;
	sd[5].sensor_scanning_enabled = 1;
	sd[5].event_messages_enabled = 1;
	sd[5].unavailable = 0;
      }
      else if (i==20) {
	sd[6].last_sensor_reading = tb;
	sd[6].sensor_scanning_enabled = 1;
	sd[6].event_messages_enabled = 1;
	sd[6].unavailable = 0;
      }
      else if (i==32) {
	sd[7].last_sensor_reading = tb;
	sd[7].sensor_scanning_enabled = 1;
	sd[7].event_messages_enabled = 1;
	sd[7].unavailable = 0;
      }
      else {
	if (t>temp_f)
	  temp_f=t;
      }

    }
    tb = temp_f;
    sd[8].last_sensor_reading = tb;
    sd[8].sensor_scanning_enabled = 1;
    sd[8].event_messages_enabled = 1;
    sd[8].unavailable = 0;

    //trigger a new monitoring cycle
    i2c_read(i2c_fd_snsr[OCTOPUS_I2C_BUS], MACHXO2_ADDR, 142, &lsb);
  }
  else if(ready) {
    //not powered up
    u8 i;
    for (i=4;i<9;++i) {
      sd[i].last_sensor_reading = 0;
      sd[i].sensor_scanning_enabled = 0;
      sd[i].event_messages_enabled = 0;
      sd[i].unavailable = 1;
			sd[i].current_state_mask = 0;
    }

  }
  unlock(2);

}



/*==============================================================*/
/* 		VUP Temp Sensor											*/
/*==============================================================*/
void read_sensor_temp_vup_remote(void) {
  //Reading has been done. Deal only with messages here
    //	Debug info
    char line[1000];
    u8 sensor_N = 5;
    if (check_power_up()) {
        static int first_time = 1;
        static int up_noncrt_assert = 0;
        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
						sd[sensor_N].current_state_mask = 0xE0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for VUP temperature sensor");
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
						sd[sensor_N].current_state_mask = 0xD0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for VUP temperature sensor");
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
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
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
}

/*==============================================================*/
/* 		K7 Temp Sensor											*/
/*==============================================================*/
void read_sensor_temp_k7_remote(void) {
	char line[1000];
	u8 sensor_N = 6;
    if (check_power_up()) {
        static int first_time = 1;
        static int up_noncrt_assert = 0;

        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
						sd[sensor_N].current_state_mask = 0xE0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for K7 temperature sensor");
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
						sd[sensor_N].current_state_mask = 0xD0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for K7 temperature sensor");
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
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
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
}

/*==============================================================*/
/* 		RAIL_2V7_INTERMEDIATE Local=0 Temp Sensor											*/
/*==============================================================*/
void read_sensor_temp_rail_2v7_intermediate_remote(void) {
	char line[1000];
	u8 sensor_N = 7;
    if (check_power_up()) {
        static int first_time = 1;
        static int up_noncrt_assert = 0;
        static int up_crt_assert = 0;
        static int up_nonrec_assert = 0;
        if (first_time) {
            first_time = 0;
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
						sd[sensor_N].current_state_mask = 0xE0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for Intermediate Rail temperature sensor");
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
						sd[sensor_N].current_state_mask = 0xD0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for Intermediate Rail temperature sensor");
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
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
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
}

/*==============================================================*/
/* 		MGT_VUP Local=0 Temp Sensor								*/
/*==============================================================*/
void read_sensor_temp_mgt_vup_remote(void) {
	char line[1000];
	u8 sensor_N = 8;
	if (check_power_up()) {
	  static int first_time = 1;
	  static int up_noncrt_assert = 0;
	  if (first_time) {
            first_time = 0;
	  } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_non_recoverable_threshold) {
            // Transition to M6 for non-recoverable
						sd[sensor_N].current_state_mask = 0xE0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for MGT VUP Rails temperature sensor");
	  } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
						sd[sensor_N].current_state_mask = 0xD0;
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for MGT VUP Rails temperature sensor");
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
            up_noncrt_assert = 1;
	  } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
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
}





/*==============================================================*/
/* 		QSFPDD Sensor				                        */
/*==============================================================*/
void read_sensor_optics(void) {
    lock(2);

    char line[1000];
    int res = 0;
    //	Sensor Data Record
    u8 sensor_N = 9;

    if (check_power_up()) {

      //Switch the LEDs if there is a problem
      int mask = optics_presence(i2c_fd_snsr[OCTOPUS_I2C_BUS]);
      u8 i=0;
	for (i=0;i<15;++i) {
	  u8 wasThere = (optics_powered&(1<<i))==0;
	  u8 isThere  = (mask&(1<<i))==0;
	  //if problem appeared set LED state to point to the issue
	  if ((optics_powered&0x7fff)!=(mask&0x7fff)) {
	    if (wasThere &isThere)
	      set_led(i2c_fd_snsr[OCTOPUS_I2C_BUS],i,0,1,0);
	    else if (wasThere &!isThere)
	      set_led(i2c_fd_snsr[OCTOPUS_I2C_BUS],i,2,0,0);
	    else if (isThere&!wasThere)
	      set_led(i2c_fd_snsr[OCTOPUS_I2C_BUS],i,0,0,1);
	    else if ((!isThere)&(!wasThere))
	      set_led(i2c_fd_snsr[OCTOPUS_I2C_BUS],i,0,0,0);
	  }
	  else {
	      set_led(i2c_fd_snsr[OCTOPUS_I2C_BUS],i,3,3,3);
	  }
	}

        float temp_f = optics_temperature(i2c_fd_snsr[OCTOPUS_I2C_BUS],mask);
        //	Convert float to byte and get precision
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
            unlock(2);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Non-recoverable threshold crossed for QSFPDD temperature sensor");
            lock(2);
        } else if (sd[sensor_N].last_sensor_reading >= sdr[sensor_N].upper_critical_threshold) {
            // Transition to M6 for upper critical
						sd[sensor_N].current_state_mask = 0xD0;
            unlock(2);
            picmg_m6_state(fru_inventory_cache[0].fru_dev_id);
            logger("WARNING","Critical threshold crossed for QSFPDD temperature sensor");
            lock(2);
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
            up_noncrt_assert = 1;
        } else if (up_noncrt_assert == 1 && sd[sensor_N].last_sensor_reading < sdr[sensor_N].upper_non_critical_threshold) {
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
    unlock(2);
}
