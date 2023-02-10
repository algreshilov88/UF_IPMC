/*
    UF_IPMC/i2c.c

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
#define uchar unsigned char

#include "timer.h"
#include "ipmi.h"
#include "ws.h"
#include "i2c.h"
#include "debug.h"
#include "ipmc.h"
#include "logger.h"
#include "toml.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#define MAX_DELIVERY_ATTEMPTS 3
#define I2C_MASTER_COUNT 	  1

#define MAP_SIZE 4096UL
//#define MAP_MASK (MAP_SIZE - 1)

#define base_slave_0 	0x40000000
#define offset_slave_1 	0x2000000

#define base_addr	0x00

#define set_MS 	0x1200000

/*==============================================================*/
/* Local Variables						*/
/*==============================================================*/
unsigned	i2c_last_channel_used = 1;

void ( *i2c_slave_receive_callback )( void *, int ) = 0;

/*==============================================================*/
/* Global Variables						*/
/*==============================================================*/
extern unsigned long long int lbolt;
unsigned int channel;
unsigned int local_i2c_address;
unsigned int remote_i2c_address;	/* used for debugging only */

/*==============================================================*/
/* Local Function Prototypes					*/
/*==============================================================*/
void i2c_master_complete( void *ws, unsigned int status );
void i2c_slave_complete( void *ws, unsigned int status );

/*==============================================================
 * i2c_initialize()
 *==============================================================*/
int i2c_fd;
int devmem_fd;
void *devmem_ptr;

void i2c_initialize( void )
{
	toml_table_t* config;
	toml_table_t* adptr_num;
	toml_array_t* arr;
	toml_raw_t raw;
	long long int i2c_cadence;
	char I2C_ADAPTER[20];

	FILE* fp;
	char errbuf[1000];

	/* Open the file. */
	if (0 == (fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r"))) {
		fp = fopen("/root/UF_IPMC/CONFIG/CONFIG.toml", "r");
		logger("ERROR", "fopen() in i2c_initialize() (CONFIG.toml)");
	}

	/* Run the files through the parser. */
	config = toml_parse_file(fp, errbuf, sizeof(errbuf));
	if (0 == config) {
		logger("ERROR", "toml_parse_file() in i2c_initialize() (CONFIG.toml)");
		if (toml_parse_file(fp, errbuf, sizeof(errbuf)) == 0) {
			exit(EXIT_FAILURE);
		}
	}

	if (fclose(fp) < 0) {
		fclose(fp);
		perror("fclose() failed");
	}

	/* Locate the [I2C_ADAPTERS] table. */
	if (0 == (adptr_num = toml_table_in(config, "I2C_ADAPTERS"))) {
		logger("ERROR", "toml_table_in() 'I2C_ADAPTERS' in i2c_initialize()");
	}

	/* Extract 'i2c_cadence' config value. */
	if (0 == (raw = toml_raw_in(adptr_num, "i2c_cadence"))) {
		logger("ERROR", "toml_raw_in() 'i2c_cadence' in i2c_initialize()");
	}

	/* Convert the raw value into an int. */
	if (toml_rtoi(raw, &i2c_cadence)) {
		logger("ERROR", "toml_rtoi() 'i2c_cadence' in i2c_initialize()");
	}

	snprintf(I2C_ADAPTER, 19, "/dev/i2c-%d", (int) i2c_cadence);

	if ((i2c_fd = open(I2C_ADAPTER, O_RDWR | O_NONBLOCK)) < 0)
  {
      //perror("open /dev/i2c-cadence");
			logger("open /dev/i2c-cadence", strerror(errno));
  }
  else{printf("/dev/i2c-cadence opened \n");}

  if ((devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
  {
      //perror("open dev/mem");
			logger("open dev/mem", strerror(errno));
  }
  else{printf("/dev/mem opened \n");}

  devmem_ptr = mmap(NULL,0x16000 * MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, devmem_fd, base_slave_0);

  if (devmem_ptr == MAP_FAILED)
  {
      //printf("devmem mmap call failure.\n");
			logger("ERROR", "devmem mmap call failure");
      return;
  }

	local_i2c_address = module_get_i2c_address( I2C_ADDRESS_LOCAL );
	remote_i2c_address =  module_get_i2c_address( I2C_ADDRESS_REMOTE );

	toml_free(config);
}

void i2c_deinitialize( void )
{
	if ((close(i2c_fd)) < 0)
  {
		//perror("Close i2c_fd_0");
		logger("Close i2c_fd", strerror(errno));
  }

	if ((close(devmem_fd)) < 0)
  {
		//perror("Close devmem_fd");
		logger("Close devmem_fd", strerror(errno));
  }

  if ((munmap(devmem_ptr, 0x16000 * MAP_SIZE)) <0)
  {
		//perror("munmap devmem_ptr");
		logger("munmap devmem_ptr", strerror(errno));
  }
}

//inline
void
reg_write(void *reg_base, unsigned int offset, unsigned int value)
{
    *((volatile unsigned *)(reg_base + offset)) = value;
}

//inline
unsigned int
reg_read(void *reg_base, unsigned int offset)
{
    return *((volatile unsigned *)(reg_base + offset));
}

void
i2c_slave_read_0( void )
{
    IPMI_WS *ws;
    unsigned int reg = 0x00;
    unsigned int pkg_size;
    unsigned int i;
		unsigned int ret;

		ret = reg_read(devmem_ptr, set_MS);
		ret &= ~0x01;
    reg_write(devmem_ptr, set_MS, ret);

    dputstr( DBG_I2C | DBG_INOUT, "i2c_slave_read_0: ingress\n" );
    if( !( ws = ws_alloc() ) ) {
        dputstr( DBG_I2C | DBG_ERR, "i2c_slave_read_0: ws allocation failed\n" );
        return;
    }

    /* fill in the ws struct */
    ws->outgoing_protocol = IPMI_CH_PROTOCOL_IPMB;
    ws->incoming_protocol = IPMI_CH_PROTOCOL_IPMB;
    ws->outgoing_medium = IPMI_CH_MEDIUM_IPMB;
    ws->incoming_medium = IPMI_CH_MEDIUM_IPMB;
    ws->outgoing_channel = IPMI_CH_NUM_PRIMARY_IPMB;
    ws->addr_out = remote_i2c_address;
    ws->ipmi_completion_function = i2c_slave_receive_callback;

    pkg_size = reg_read(devmem_ptr, base_addr);

    if (pkg_size != 0)
    {
       	ws->len_in = pkg_size;
       	for (i=0; i<pkg_size-1; i++)
       	{
           	reg = reg + 0x004;
           	ws->pkt_in[i] = reg_read(devmem_ptr, reg);
	   		ws->i2c_channel = 0;
	   		//printf("data_rcv_0 = 0x%x\n\r", (unsigned int) ws->pkt_in[i]);
       	}

				i2c_slave_complete( (void *)ws, 0 );

    } else {
				i2c_slave_complete( (void *)ws, 1 );
    }

	// TODO fix this code, this is a hack.
    if( ws->interface == 1 )
        channel = i2c_last_channel_used = ws->interface;

    //usleep(150000);
}

void
i2c_slave_read_1( void )
{
    IPMI_WS *ws;
    unsigned int reg = 0x2000000;
    unsigned int pkg_size;
    unsigned int i;
		unsigned int ret;

		ret = reg_read(devmem_ptr, set_MS);
		ret &= ~0x01;
    reg_write(devmem_ptr, set_MS, ret);

    dputstr( DBG_I2C | DBG_INOUT, "i2c_slave_read_0: ingress\n" );
    if( !( ws = ws_alloc() ) ) {
        dputstr( DBG_I2C | DBG_ERR, "i2c_slave_read_0: ws allocation failed\n" );
        return;
    }

    /* fill in the ws struct */
    ws->outgoing_protocol = IPMI_CH_PROTOCOL_IPMB;
    ws->incoming_protocol = IPMI_CH_PROTOCOL_IPMB;
    ws->outgoing_medium = IPMI_CH_MEDIUM_IPMB;
    ws->incoming_medium = IPMI_CH_MEDIUM_IPMB;
    ws->outgoing_channel = IPMI_CH_NUM_PRIMARY_IPMB;
    ws->addr_out = remote_i2c_address;
    ws->ipmi_completion_function = i2c_slave_receive_callback;

    pkg_size = reg_read(devmem_ptr, offset_slave_1);

    if (pkg_size != 0)
    {
       	ws->len_in = pkg_size;
       	for (i=0; i<pkg_size-1; i++)
       	{
           	reg = reg + 0x004;
           	ws->pkt_in[i] = reg_read(devmem_ptr, reg);
	   				ws->i2c_channel = 1;
	   				//printf("data_rcv_1 = 0x%x\n\r", (unsigned int) ws->pkt_in[i]);
       	}

		i2c_slave_complete( (void *)ws, 0 );

    } else {
				i2c_slave_complete( (void *)ws, 1 );
    }

	// TODO fix this code, this is a hack.
    if( ws->interface == 1 )
        channel = i2c_last_channel_used = ws->interface;

    //usleep(150000);
}

/*
 * i2c_master_write()
 * 	Gets called by ws_process_work_list() when we find a ws with state
 * 	WS_ACTIVE_MASTER_WRITE_PENDING in the work list.
 */
int
i2c_master_write_0( IPMI_WS *ws )
{
    long funcs;
    unsigned int i, n;
		unsigned int ret;
		unsigned char outbuf[ws->len_out];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    if (ioctl(i2c_fd, I2C_FUNCS, &funcs) < 0)
    {
        //perror("ioctl(I2C_FUNCS) in i2c_master_write_0()");
				logger("ioctl(I2C_FUNCS) in i2c_master_write_0()", strerror(errno));
    }

    assert(funcs & I2C_FUNC_I2C);

    dputstr( DBG_I2C | DBG_INOUT, "i2c_master_write_0(): ingress\n" );

    /* fill in the ws struct & the data buf */
    ws->outgoing_protocol = IPMI_CH_PROTOCOL_IPMB;
    ws->outgoing_medium = IPMI_CH_MEDIUM_IPMB;
    ws->outgoing_channel = IPMI_CH_NUM_PRIMARY_IPMB;
    ws->addr_out = remote_i2c_address;

    msgs[0].addr = remote_i2c_address/2;
    msgs[0].flags = 0;
    msgs[0].len = ws->len_out;
    /*if ((msgs[0].buf = (unsigned char *)malloc(ws->len_out+1)) == NULL)
		{
				logger("ERROR", "malloc() failed in i2c_master_write_0()");
				exit(EXIT_FAILURE);
		}*/

    for (i=0; i<=ws->len_out; i++) {
        //msgs[0].buf[i] = ws->pkt_out[i];
				outbuf[i] = ws->pkt_out[i];
    }

		msgs[0].buf = outbuf;

    for (n=0; n<=ws->len_out; n++) {
    		//printf("data_sent_0 = %x\n\r", (unsigned int) msgs[0].buf[n]);
    }

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

		ret = reg_read(devmem_ptr, set_MS);
		ret |= 0x01;
    reg_write(devmem_ptr, set_MS, ret);

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0)
    {
				ret = reg_read(devmem_ptr, set_MS);
				ret &= ~0x01;
	    	reg_write(devmem_ptr, set_MS, ret);

        perror("ioctl(I2C_RDWR) in i2c_master_write_0()");

				//logger("i2c-cadence master write error", strerror(errno));

				//usleep(100000);

				/*if (errno != EBUSY && errno != EAGAIN) {
						if (msgs[0].buf != NULL)
						{
								free(msgs[0].buf);
								msgs[0].buf = NULL;
						}

						i2c_master_write_0( ws );
				} else {*/
						/*if (msgs[0].buf != NULL)
						{
								free(msgs[0].buf);
								msgs[0].buf = NULL;
						}*/

						//i2c_master_write_1( ws );
						return -1;
				//}
    } else {
				ret = reg_read(devmem_ptr, set_MS);
				ret &= ~0x01;
				reg_write(devmem_ptr, set_MS, ret);

				if (ws->pkt_out[0] == 0x14 && ws->pkt_out[4] == 0x00) {
					logger("Event Message","Set Event Receiver(Events Rearming)");
				}

				i2c_master_complete( (void *)ws, 0 );
		//printf("i2c_msg_sent \n");
    }

		/*if (msgs[0].buf != NULL)
		{
				free(msgs[0].buf);
				msgs[0].buf = NULL;
		}*/
		return 0;
}

int
i2c_master_write_1( IPMI_WS *ws )
{
    long funcs;
    unsigned int i, n;
		unsigned int ret;
		unsigned char outbuf[ws->len_out];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    if (ioctl(i2c_fd, I2C_FUNCS, &funcs) < 0)
    {
        //perror("ioctl(I2C_FUNCS) in i2c_master_write");
				logger("ioctl(I2C_FUNCS) in i2c_master_write_1()", strerror(errno));
    }

    assert(funcs & I2C_FUNC_I2C);

    dputstr( DBG_I2C | DBG_INOUT, "i2c_master_write_1(): ingress\n" );

    /* fill in the ws struct & the data buf */
    ws->outgoing_protocol = IPMI_CH_PROTOCOL_IPMB;
    ws->outgoing_medium = IPMI_CH_MEDIUM_IPMB;
    ws->outgoing_channel = IPMI_CH_NUM_PRIMARY_IPMB;
    ws->addr_out = remote_i2c_address;

    msgs[0].addr = remote_i2c_address/2;
    msgs[0].flags = 0;
    msgs[0].len = ws->len_out;
    /*if ((msgs[0].buf = (unsigned char *)malloc(ws->len_out+1)) == NULL)
		{
				logger("ERROR", "malloc() failed in i2c_master_write_1()");
				exit(EXIT_FAILURE);
		}*/

    for (i=0; i<=ws->len_out; i++) {
        //msgs[0].buf[i] = ws->pkt_out[i];
				outbuf[i] = ws->pkt_out[i];
    }

		msgs[0].buf = outbuf;

    for (n=0; n<=ws->len_out; n++) {
    		//printf("data_sent_1 = %x\n\r", (unsigned int) msgs[0].buf[n]);
    }

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

		ret = reg_read(devmem_ptr, set_MS);
		ret |= 0x02;
    reg_write(devmem_ptr, set_MS, ret);

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0)
    {
				ret = reg_read(devmem_ptr, set_MS);
				ret &= ~0x02;
				reg_write(devmem_ptr, set_MS, ret);

        perror("ioctl(I2C_RDWR) in i2c_master_write_1()");

				//logger("i2c-xilinx master write error", strerror(errno));

				//usleep(100000);

				/*if (errno != EBUSY && errno != EAGAIN) {
						if (msgs[0].buf != NULL)
						{
								free(msgs[0].buf);
								msgs[0].buf = NULL;
						}

						i2c_master_write_1( ws );
				} else {*/
						/*if (msgs[0].buf != NULL)
						{
								free(msgs[0].buf);
								msgs[0].buf = NULL;
						}*/

						//i2c_master_write_0( ws );
						return -1;
				//}
    } else {
				ret = reg_read(devmem_ptr, set_MS);
				ret &= ~0x02;
				reg_write(devmem_ptr, set_MS, ret);

				if (ws->pkt_out[0] == 0x14 && ws->pkt_out[4] == 0x00) {
					logger("Event message","Set Event Receiver(Events Rearming)");
				}
				i2c_master_complete( (void *)ws, 0 );
				//printf("i2c_msg_sent \n");
    }

		/*if (msgs[0].buf != NULL)
		{
				free(msgs[0].buf);
				msgs[0].buf = NULL;
		}*/
		return 0;
}

void
i2c_master_write( IPMI_WS *ws ) {
		int ret = -1;
		int count;
		for (count = 0; count < I2C_MASTER_COUNT; count++)
		{
				ret = i2c_master_write_0( ws );

				if (ret < 0)
				{
						ret = i2c_master_write_1( ws );
				}
		}
}

/* Master op transport completion routine */
void
i2c_master_complete( void *ws, unsigned int status )
{
		switch( status ) {
        case 0:
            dputstr( DBG_I2C | DBG_LVL1, "i2c_master_complete: completed with status 0\n" );
						ws_set_state( ((IPMI_WS *)(ws)), WS_ACTIVE_MASTER_WRITE_SUCCESS );
						if( ((IPMI_WS *)(ws))->ipmi_completion_function ) {
								( ((IPMI_WS *)(ws))->ipmi_completion_function )( (void *)ws,
								XPORT_REQ_NOERR );
						} else {
								ws_free( ((IPMI_WS *)(ws)) );
						}
						break;
        case 1:
            dputstr( DBG_I2C | DBG_ERR, "i2c_master_complete: completed with status 1\n" );
						((IPMI_WS *)(ws))->delivery_attempts++;

						/* if status indicates a retryable error and we have not
			 			* exceeded max number of deliveries, try again */
						/* back to the queue */
						if( ( WS_ACTIVE_MASTER_WRITE_PENDING == ((IPMI_WS *)(ws))->ws_state )
						&& ( ((IPMI_WS *)(ws))->delivery_attempts < MAX_DELIVERY_ATTEMPTS ) )
						{
								ws_set_state( ((IPMI_WS *)(ws)), WS_ACTIVE_MASTER_WRITE );
            } else {
								/* invalid state or exceeded retries - return error to upper layer */
								if( ((IPMI_WS *)(ws))->ipmi_completion_function ) {
										(((IPMI_WS *)(ws))->ipmi_completion_function)( (void *)ws,
										XPORT_REQ_ERR );
								} else {
										ws_free( ((IPMI_WS *)(ws)) );
								}
						}
						break;
	}
}

/* Slave completion routine */
void
i2c_slave_complete( void *ws, unsigned int status )
{
		switch( status ) {
        case 0:
						((IPMI_WS *)(ws))->addr_out = remote_i2c_address;
						ws_set_state( ((IPMI_WS *)(ws)), WS_ACTIVE_IN );
						break;
        case 1:
						ws_free( ((IPMI_WS *)(ws)) );
						break;
		}
}

/* set callback function for slave receive */
void
i2c_set_slave_receive_callback( void ( *callback_fn )( void *, int ) )
{
    i2c_slave_receive_callback = callback_fn;
}
