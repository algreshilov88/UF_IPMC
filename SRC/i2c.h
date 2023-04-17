/*
    UF_IPMC/i2c.h

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
/*==============================================================*/
/* I2C 7-BIT ADDRESSING 					*/
/*==============================================================*/

#define I2C_ADDRESS_LOCAL	0
#define I2C_ADDRESS_REMOTE	1

extern int repeat;
extern int i2c_fd_0;
extern int i2c_fd_1;
extern int devmem_fd;
extern void *devmem_ptr;

/*==============================================================*/
/* Function Prototypes						*/
/*==============================================================*/
void i2c_initialize( void );
void i2c_deinitialize( void );

int i2c_master_write_0( IPMI_WS *ws );
int i2c_master_write_1( IPMI_WS *ws );
void i2c_master_write( IPMI_WS *ws );
void i2c_slave_read_0( void );
void i2c_slave_read_1( void );
void i2c_set_slave_receive_callback( void ( *callback_fn )( void *, int ) );
void reg_write(void *reg_base, unsigned int offset, unsigned int value);
unsigned int reg_read(void *reg_base, unsigned int offset);
