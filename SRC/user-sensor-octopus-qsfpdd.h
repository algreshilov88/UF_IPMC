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
void user_sensor_state_poll(void);
void user_module_sensor_init(void);
void semaphore_initialize(void);


void read_all_i2c_sensors(void);
void read_sensor_temp_vup_remote(void);
void read_sensor_temp_k7_remote(void);
void read_sensor_temp_rail_2v7_intermediate_remote(void);
void read_sensor_temp_mgt_vup_remote(void);
void read_sensor_optics(void);

