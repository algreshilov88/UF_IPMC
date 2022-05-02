/*
    UF_IPMC/i2c-sensor.c

    Original work Copyright (C) 2017 JamesDunne
    https://gist.github.com/JamesDunne

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
#include "toml.h"
#include "logger.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#define MAX_SENSOR_COUNT	20

typedef unsigned char   u8;

/*==============================================================
 * i2c_sensor_initialize()
 *==============================================================*/
int i2c_fd_snsr[MAX_SENSOR_COUNT];

void i2c_sensor_initialize( void )
{
	toml_table_t* config;
	toml_table_t* adptr_snsr_num;
	toml_array_t* arr;
	toml_raw_t raw;

	int n;

	FILE* fp;
	char errbuf[1000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r");
		logger("ERROR", "fopen() in i2c_sensor_initialize() (CONFIG.toml)");
	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in i2c_sensor_initialize() (CONFIG.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [I2C_SENSORS] table. */
	if (0 == (adptr_snsr_num = toml_table_in(config, "I2C_SENSORS"))) {
		logger("ERROR", "toml_table_in() 'I2C_SENSORS' in i2c_sensor_initialize()");
	}

	/* Extract 'i2c_sensor' config array. */
	if (0 == (arr = toml_array_in(adptr_snsr_num, "i2c_sensor"))) {
		logger("ERROR", "toml_array_in() 'i2c_sensor' in i2c_sensor_initialize()");
	}

	long long int i2c_sensor[toml_array_nelem(arr)];
	char I2C_ADAPTER_SNSR[toml_array_nelem(arr)][20];

	for (n=0; n<toml_array_nelem(arr); n++) {
		/* Extract 'i2c_sensor[n]' config value. */
		if (0 == (raw = toml_raw_at(arr, n))) {
			logger("ERROR", "toml_raw_at() 'i2c_sensor[n]' in i2c_sensor_initialize()");
		};

		/* Convert the raw value into an int. */
		if (toml_rtoi(raw, &i2c_sensor[n])) {
			logger("ERROR", "toml_rtoi() 'i2c_sensor[n]' in i2c_sensor_initialize()");
		};

		snprintf(I2C_ADAPTER_SNSR[n], 19, "/dev/i2c-%d", (int) i2c_sensor[n]);

    		if ((i2c_fd_snsr[n] = open(I2C_ADAPTER_SNSR[n], O_RDWR | O_NONBLOCK)) < 0) {
			i2c_fd_snsr[n] = open(I2C_ADAPTER_SNSR[n], O_RDWR | O_NONBLOCK);
			logger(I2C_ADAPTER_SNSR[n], strerror(errno));
		}
	    	else{printf("/dev/i2c-%d opened \n", (int) i2c_sensor[n]);}
	}

	toml_free(config);
}

void i2c_sensor_deinitialize( void )
{
	toml_table_t* config;
	toml_table_t* adptr_snsr_num;
    	toml_array_t* arr;
	toml_raw_t raw;

	int n;

	FILE* fp;
	char errbuf[1000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r");
		logger("ERROR", "fopen() in i2c_sensor_deinitialize() (CONFIG.toml)");
	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in i2c_sensor_deinitialize() (CONFIG.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [I2C_SENSORS] table. */
	if (0 == (adptr_snsr_num = toml_table_in(config, "I2C_SENSORS"))) {
		logger("ERROR", "toml_table_in() 'I2C_SENSORS' in i2c_sensor_deinitialize()");
	}

	/* Extract 'i2c_sensor' config array. */
	if (0 == (arr = toml_array_in(adptr_snsr_num, "i2c_sensor"))) {
		logger("ERROR", "toml_array_in() 'i2c_sensor' in i2c_sensor_deinitialize()");
	}

	long long int i2c_sensor[toml_array_nelem(arr)];
	char I2C_ADAPTER_SNSR[toml_array_nelem(arr)][20];

	for (n=0; n<toml_array_nelem(arr); n++) {

		/* Extract 'i2c_sensor[n]' config value. */
		if (0 == (raw = toml_raw_at(arr, n))) {
			logger("ERROR", "toml_raw_at() 'i2c_sensor[n]' in i2c_sensor_deinitialize()");
		};

		/* Convert the raw value into an int. */
		if (toml_rtoi(raw, &i2c_sensor[n])) {
			logger("ERROR", "toml_rtoi() 'i2c_sensor[n]' in i2c_sensor_deinitialize()");
		};

		snprintf(I2C_ADAPTER_SNSR[n], 19, "/dev/i2c-%d", (int) i2c_sensor[n]);

		if ((close(i2c_fd_snsr[n])) < 0) {
			close(i2c_fd_snsr[n]);
			logger(I2C_ADAPTER_SNSR[n], strerror(errno));
		}
	}
}

// Write to an I2C slave device's register:
int i2c_write(int i2c_fd_snsr, u8 slave_addr, u8 reg, u8 data) {
    long funcs;
    u8 outbuf[2];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    if (ioctl(i2c_fd_snsr, I2C_FUNCS, &funcs) < 0) {
				//perror("ioctl(I2C_FUNCS) in i2c_write");
				logger("ioctl(I2C_FUNCS) in i2c_write()", strerror(errno));
				return (-1);
    }

    assert(funcs & I2C_FUNC_I2C);

    outbuf[0] = reg;
    outbuf[1] = data;

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(i2c_fd_snsr, I2C_RDWR, &msgset) < 0) {
        //perror("ioctl(I2C_RDWR) in i2c_write");
				logger("ioctl(I2C_RDWR) in i2c_write()", strerror(errno));
        return (-1);
    }

    return 0;
}

// Read the given I2C slave device's register and return the read value in `*result`:
int i2c_read(int i2c_fd_snsr, u8 slave_addr, u8 reg, u8 *result) {
    long funcs;
    u8 outbuf[1], inbuf[1];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    if (ioctl(i2c_fd_snsr, I2C_FUNCS, &funcs) < 0) {
				//perror("ioctl(I2C_FUNCS) in i2c_read");
				logger("ioctl(I2C_FUNCS) in i2c_read()", strerror(errno));
				return (-1);
    }

    assert(funcs & I2C_FUNC_I2C);


    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = slave_addr;
    //msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = inbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    outbuf[0] = reg;

    inbuf[0] = 0;

    *result = 0;
    if (ioctl(i2c_fd_snsr, I2C_RDWR, &msgset) < 0) {
        //perror("ioctl(I2C_RDWR) in i2c_read");
				logger("ioctl(I2C_RDWR) in i2c_read()", strerror(errno));
        return (-1);
    }

    *result = inbuf[0];
    return 0;
}

// Write into PI4IOE5V96224ZLEX register, 3 data bytes and no address
int i2c_write_qsfp2(int i2c_fd_bus, u8 i2c_chip_addr, u8 data[3])
{
    long funcs;
    u8 outbuf[3];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    if (ioctl(i2c_fd_bus, I2C_FUNCS, &funcs) < 0)
		{
		//perror("ioctl(I2C_FUNCS) in i2c_write_qsfp2()");
		logger("ioctl(I2C_FUNCS) in i2c_write_qsfp2()", strerror(errno));
		return (-1);
		}

    assert(funcs & I2C_FUNC_I2C);

    outbuf[0] = data[0];
    outbuf[1] = data[1];
    outbuf[2] = data[2];

    msgs[0].addr = i2c_chip_addr;
    msgs[0].flags = 0;
    msgs[0].len = 3;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(i2c_fd_bus, I2C_RDWR, &msgset) < 0) {
        //perror("ioctl(I2C_RDWR) in i2c_write_qsfp2()");
				logger("ioctl(I2C_RDWR) in i2c_write_qsfp2()", strerror(errno));
        return (-1);
    }

    return 0;
}

/* TO FIX
int i2c_read_qsfp2(int i2c_fd_bus, u8 i2c_chip_addr, u8 *result) {
    long funcs;
    u8 inbuf[3];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    if (ioctl(i2c_fd_bus, I2C_FUNCS, &funcs) < 0)
		{
		//perror("ioctl(I2C_FUNCS) in i2c_read_qsfp2()");
		logger("ioctl(I2C_FUNCS) in i2c_read_qsfp2()", strerror(errno));
		return (-1);
		}

    assert(funcs & I2C_FUNC_I2C);


    msgs[0].addr = i2c_chip_addr;
    //msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[0].flags = I2C_M_RD;
    msgs[0].len = 3;
    msgs[0].buf = inbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    inbuf[0] = 0;
    inbuf[1] = 0;
    inbuf[2] = 0;

    *result = 0;
    if (ioctl(i2c_fd_bus, I2C_RDWR, &msgset) < 0) {
    		// perror("ioctl(I2C_RDWR) in i2c_read_qsfp2()");
				logger("ioctl(I2C_FUNCS) in i2c_read_qsfp2()", strerror(errno));
        return (-1);
    }

    result[0] = inbuf[0];
    result[1] = inbuf[1];
    result[2] = inbuf[2];
    return 0;
}
*/
