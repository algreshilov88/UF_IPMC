/*
    UF_IPMC/ipmc.h

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
void sig_handler(int signo);
void ipmb_buffers_enable( void );
void ipmb_buffers_disable( void );
void module_init( void );
void module_cold_reset( unsigned char dev_id );
void module_warm_reset( unsigned char dev_id );
void module_graceful_reboot( unsigned char dev_id );
void module_issue_diag_int( unsigned char dev_id );
void module_quiesce( unsigned char dev_id );
void module_event_handler( IPMI_PKT *pkt );
unsigned char module_get_i2c_address( int address_type );
void module_term_process( unsigned char * );
void module_led_on( unsigned led_state );
void module_led_off( unsigned led_state );
void module_process_response( IPMI_WS *req_ws, unsigned char seq, unsigned char completion_code );
void module_sensor_init( void );
void module_rearm_events( void );
void watchdog_init( void );
void fru_data_init( void );
void read_fru_hot_swap( void );
void read_ipmb_0_status( void );
void read_hot_swap_handle( void );
void watchdog_ping( void );
