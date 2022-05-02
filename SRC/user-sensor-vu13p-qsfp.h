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
typedef long long int u64;
typedef unsigned int u32;

// this structure defines position of the qsfp select bit within I2C expansion registers
typedef struct qsfp_select_s
{
    unsigned char chip_ind; // 0 or 1
    unsigned char byte_num; // 0,1,2
    unsigned char bit_num; // 0..7 within byte
} qsfp_select_t;

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

void user_sensor_state_poll(void);
void user_module_sensor_init(void);
void semaphore_initialize(void);
void read_sensor_temp_qsfp_max(void);
