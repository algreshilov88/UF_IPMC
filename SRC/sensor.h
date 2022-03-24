/*
    UF_IPMC/sensor.h

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
//#define BF_MS_FIRST

typedef struct sensor_data {
	uchar	sensor_id;
	uchar	last_sensor_reading;
	uchar	scan_period;		/* time between each sensor scan in seconds, 0 = no scan */
	void(*scan_function)( void );	/* the routine that does the sensor scan */
#ifdef BF_MS_FIRST
	uchar	event_messages_enabled:1,	/* 0b = All Event Messages disabled from this sensor */
		sensor_scanning_enabled:1,	/* 0b = sensor scanning disabled */
		unavailable:1,			/* 1b = reading/state unavailable */
		reserved;
#else
	uchar	reserved:5,
		unavailable:1,
		sensor_scanning_enabled:1,
		event_messages_enabled:1;
#endif
	uchar	std_ipmi_byte;
	uchar	current_state_mask;
#ifdef BF_MS_FIRST
	uchar	ipmb_b_override_state:1,	/* [7]  IPMB B Override State
						   0b = Override state, bus isolated
						   1b = Local Control state - IPM
						   Controller determines state of bus. */
		ipmb_b_local_status:3,		/* [6:4] = IPMB B Local Status */
		ipmb_a_override_state:1,	/* [3]  IPMB A Override State
						   0b = Override state, bus isolated
						   1b = Local Control state - IPM
						   Controller determines state of bus. */
		ipmb_a_local_status:3;		/* [2:0] = IPMB A Local Status (see IPMB_STATUS above) */
#else
	uchar	ipmb_a_local_status:3,
		ipmb_a_override_state:1,
		ipmb_b_local_status:3,
		ipmb_b_override_state:1;
#endif
#ifdef BF_MS_FIRST
	uchar	:4,					/* [7:4]  Reserved. Write as 0h, ignore on read */
		ipmb_a_enabled_ipmb_b_enabled:1,	/* [3] 1b = IPMB A enabled, IPMB-B enabled */
		ipmb_a_disabled_ipmb_b_enabled:1,	/* [2] 1b = IPMB A disabled, IPMB-B enabled */
		ipmb_a_enabled_ipmb_b_disabled:1,	/* [1] 1b = IPMB-A enabled, IPMB-B disabled */
		ipmb_a_disabled_ipmb_b_disabled:1;	/* [0] 1b = IPMB A disabled, IPMB-B disabled */
#else
	uchar	ipmb_a_disabled_ipmb_b_disabled:1,	/* [0] 1b = IPMB A disabled, IPMB-B disabled */
		ipmb_a_enabled_ipmb_b_disabled:1,	/* [1] 1b = IPMB-A enabled, IPMB-B disabled */
		ipmb_a_disabled_ipmb_b_enabled:1,	/* [2] 1b = IPMB A disabled, IPMB-B enabled */
		ipmb_a_enabled_ipmb_b_enabled:1,	/* [3] 1b = IPMB A enabled, IPMB-B enabled */
		:4;
#endif
	uchar 	reserved2;			/* [7:0]  Optional/Reserved. If provided,
						   write as 80h (IPMI restriction). Ignore on read. */
} SENSOR_DATA;

typedef struct sdr_entry {
	unsigned short	record_id;
	uchar	rec_len;
	uchar	*record_ptr;
} SDR_ENTRY;

void ipmi_get_device_sdr_info( IPMI_PKT *pkt );
void ipmi_get_device_sdr( IPMI_PKT *pkt );
void ipmi_reserve_device_sdr_repository( IPMI_PKT *pkt );
void ipmi_get_sensor_reading( IPMI_PKT *pkt );
void get_sdr_repository_info( IPMI_PKT *pkt );
void reserve_sdr_repository( IPMI_PKT *pkt );
void clear_sdr_repository( IPMI_PKT *pkt );
void get_sensor_threshold( IPMI_PKT *pkt );
void get_sensor_hysteresis( IPMI_PKT *pkt );
void set_sensor_threshold ( IPMI_PKT *pkt );
void set_sensor_hysteresis ( IPMI_PKT *pkt );
void rearm_sensor_events ( IPMI_PKT *pkt );
void get_sensor_event_status ( IPMI_PKT *pkt );
int  mgmt_add( MGMT_CTRL_DEV_LOCATOR_RECORD *sdr );
int  sensor_add( FULL_SENSOR_RECORD *sdr, SENSOR_DATA *sensor_data );
