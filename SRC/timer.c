/*
    UF_IPMC/timer.c

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
#include "timer.h"
#include "error.h"
#include "logger.h"
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <assert.h>
#include <sys/wait.h>

#define TIMER_FREQUENCY 10000000
//#define TIMER_FREQUENCY 1

#define CQ_ARRAY_SIZE 32

unsigned long long int lbolt;

typedef struct cqe_struct {
	unsigned state;
	unsigned long long int tick;
	void *handle;
	void ( *func )( unsigned char * );
	unsigned char *arg;
} CQE;

CQE	cq_array[CQ_ARRAY_SIZE];

/*==============================================================*/
/* Function Prototypes						*/
/*==============================================================*/

void timer_process_callout_queue( void );
void cq_init( void );
CQE *cq_alloc( void );
void cq_free( CQE *cqe );
CQE *cq_get_expired_elem( unsigned long long int current_tick );
void cq_set_cqe_state( CQE *cqe, unsigned state );



void timer_expired(int sig)
{
	lbolt++;
}

/*==============================================================
 * timer_initialize()
 *==============================================================*/
/* Setup the Timer Counter 0 Interrupt */
void
timer_initialize( void )
{
    struct sigaction act;
    clockid_t clock_id;
    timer_t timer_id;
    struct sigevent clock_sig_event;
    struct itimerspec timer_value;
    int ret;

    /* Register new action for SIGUSR1 */
    memset(&act, 0, sizeof(struct sigaction));
    act.sa_handler =  timer_expired;
    ret = sigaction(SIGUSR1, &act, NULL);
    assert(ret == 0);

    clock_id = CLOCK_MONOTONIC;
    memset(&clock_sig_event, 0, sizeof( struct sigevent));
    clock_sig_event.sigev_notify = SIGEV_SIGNAL;
    clock_sig_event.sigev_signo = SIGUSR1;
    clock_sig_event.sigev_notify_attributes = NULL;
    /* Creating process interval timer */
    ret = timer_create(clock_id, &clock_sig_event, &timer_id);
    assert(ret == 0);

    /* setitng timer interval values */
    memset(&timer_value, 0, sizeof(struct itimerspec));
    timer_value.it_interval.tv_sec = 0;
    timer_value.it_interval.tv_nsec = TIMER_FREQUENCY;
//    timer_value.it_interval.tv_sec = TIMER_FREQUENCY;
//    timer_value.it_interval.tv_nsec = 0;

    /* setting timer initial expiration values*/
    timer_value.it_value.tv_sec = 0;
    timer_value.it_value.tv_nsec = TIMER_FREQUENCY;
//    timer_value.it_value.tv_sec = TIMER_FREQUENCY;
//    timer_value.it_value.tv_nsec = 0;

    /* Create timer */
    ret = timer_settime(timer_id, 0, &timer_value, NULL);
    assert(ret == 0);

	cq_init();
}

/*==============================================================
 * timer_add_callout_queue()
 *==============================================================*/
int
timer_add_callout_queue(
	void *handle,
	unsigned long long int ticks,
	void(*func)(unsigned char *),
	unsigned char *arg )
{
	CQE *cqe;

	cqe = cq_alloc();
	if( cqe ) {
		cqe->func = func;
		cqe->arg = arg;
		cqe->tick = ticks + lbolt;
		cqe->handle = handle;
		return( ESUCCESS );
	} else
		return( ENOMEM );
		perror("ENOMEM in timer_add_callout_queue()");
		logger("ERROR", "ENOMEM in timer_add_callout_queue()");
}

/*==============================================================
 * timer_remove_callout_queue()
 * 	Search the callout queue using the handle and remove the
 * 	entry if found.
 *==============================================================*/
void
timer_remove_callout_queue(
	void *handle )
{
	CQE *ptr;
	unsigned i;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
		ptr = &cq_array[i];
		if( ( ptr->state == CQE_ACTIVE ) && ( ptr->handle == handle ) ) {
			cq_free( ptr );
			break;
		}
	}
}

/*==============================================================
 * timer_reset_callout_queue()
 * 	Search the callout queue using the handle and reset the
 * 	timeout value with the new value passed in ticks.
 *==============================================================*/
void
timer_reset_callout_queue(
	void *handle,
	unsigned long long int ticks )
{
	CQE *cqe;
	unsigned i;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
		cqe = &cq_array[i];
		if( ( cqe->state == CQE_ACTIVE ) && ( cqe->handle == handle ) ) {
			cqe->tick = ticks + lbolt;
			break;
		}
	}
}


/*==============================================================
 * timer_get_expiration_time()
 * 	Get ticks to expiration.
 *
 *==============================================================*/
unsigned long long int
timer_get_expiration_time(
	void *handle )
{
	CQE *cqe;
	unsigned i;
	unsigned long long int ticks = 0;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
		cqe = &cq_array[i];
		if( ( cqe->state == CQE_ACTIVE ) && ( cqe->handle == handle ) ) {
			if( cqe->tick > lbolt )
				return( cqe->tick - lbolt );
			break;
		}
	}
	return( 0 );
}


/*==============================================================
 * timer_process_callout_queue()
 * 	Get first expired entry, invoke it's callback and remove
 * 	from queue.
 *==============================================================*/
void
timer_process_callout_queue( void )
{
	CQE *cqe;
	unsigned i;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
			cqe = &cq_array[i];
			if( ( cqe->state == CQE_ACTIVE ) && ( cqe->tick )
					&& ( lbolt >= cqe->tick ) ) {
				cq_set_cqe_state( cqe, CQE_PENDING );
				(*cqe->func)( cqe->arg );
				cq_free( cqe );
			}
	}
}

/*======================================================================*
 * CALLOUT QUEUE MANAGEMENT
 */

/*==============================================================
 * cq_init()
 *==============================================================*/
/* initialize cq structures */
void
cq_init( void )
{
	unsigned i;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
		cq_array[i].state = CQE_FREE;
	}

}

/*==============================================================
 * cq_alloc()
 *==============================================================*/
/* get a free callout queue entry (CQE) */
CQE *
cq_alloc( void )
{
	CQE *cqe = 0;
	CQE *ptr = cq_array;
	unsigned i, found = 0;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
		ptr = &cq_array[i];
		if( ptr->state == CQE_FREE ) {
			ptr->tick = 0;
			ptr->state = CQE_ACTIVE;
			cqe = ptr;
			found = 1;
			break;
		}
	}

	/*if ( !found )
	{
		cq_alloc();
	}*/
	return cqe;
}

/*==============================================================
 * cq_free()
 *==============================================================*/
/* set cqe state to free */
void
cq_free( CQE *cqe )
{
	cqe->state = CQE_FREE;
}

/*==============================================================
 * cq_get_expired_elem()
 *==============================================================*/
CQE *
cq_get_expired_elem( unsigned long long int current_tick )
{
	CQE *cqe = 0;
	CQE *ptr = cq_array;
	unsigned i;

	for ( i = 0; i < CQ_ARRAY_SIZE; i++ )
	{
		ptr = &cq_array[i];
		if( ( ptr->state == CQE_ACTIVE ) && ( ptr->tick )
				&& ( current_tick >= ptr->tick ) ) {
			cqe = ptr;
			break;
		}
	}
	return cqe;
}

/*==============================================================
 * cq_set_cqe_state()
 *==============================================================*/
void
cq_set_cqe_state( CQE *cqe, unsigned state )
{
	cqe->state = state;
}
