/*
    UF_IPMC/user-sensor.h

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
typedef unsigned int u32;

// this structure defines position of the qsfp select bit within I2C expansion registers
typedef struct qsfp_select_s
{
    unsigned char chip_ind; // 0 or 1
    unsigned char byte_num; // 0,1,2
    unsigned char bit_num; // 0..7 within byte
} qsfp_select_t;

// read temperature
union temperature
{
    short int t16;
    unsigned char  t8[2];
} t;

// read voltage
union voltage
{
    unsigned short int v16;
    unsigned char  v8[2];
} v;

//PMBus Linear Data information
typedef struct
{
    short int base : 11;
    short int mantissa : 5;
} linear11_t;

typedef union
{
    linear11_t linear;
    unsigned short int raw;
} linear11_val_t;

void user_sensor_state_poll(void);
void user_module_sensor_init(void);
void semaphore_initialize(void);
void read_sensor_pgood(void);
void read_sensor_temp_qsfp30_max(void);
void read_sensor_temp_DSE0133V2NBC(void);
void read_sensor_temp_PIM400KZ(void);
void read_sensor_temp_XADC(void);
