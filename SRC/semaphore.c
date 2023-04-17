/*
    UF_IPMC/semaphore.c

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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <errno.h>
#include "semaphore.h"
#include "logger.h"

#define MAX_DEVICES 20

union semun {
	int val;
	struct semid_ds *buf;
	unsigned short *array;
	struct seminfo *__buf;
};

int fd_semid[MAX_DEVICES];
sigset_t x;

int lock_device(int dev_ind)
{
	int sem_id = fd_semid[dev_ind];
	int nsops = 2;
	struct sembuf sops[2];

	/* wait for semaphore to reach zero */
	sops[0].sem_num = 0; /* We only use one track */
	sops[0].sem_op = 0; /* wait for semaphore flag to become zero */
	sops[0].sem_flg = 0; /* take off semaphore */

	sops[1].sem_num = 0;
	sops[1].sem_op = 1; /* increment semaphore -- take control of track */
	sops[1].sem_flg = IPC_NOWAIT; /* take off semaphore */

	if ((semop(sem_id, sops, nsops)) == -1)
	{
		if (errno != EINTR)
		{
			printf("lock semop() failed\n");
			logger("ERROR", "(LOCK) in semop() %s", strerror(errno));
			return -1;
		}
		else
		{
			lock_device(dev_ind);
		}
	}

	return 0;
}

int unlock_device(int dev_ind)
{
	int sem_id = fd_semid[dev_ind];
	int nsops = 1;
	struct sembuf sops[1];
	union semun arg;

	/* wait for semaphore to reach zero */
	sops[0].sem_num = 0;
	sops[0].sem_op = -1; /* Give UP COntrol of track */
	sops[0].sem_flg = IPC_NOWAIT; /* take off semaphore */

	if ((semop(sem_id, sops, nsops)) == -1)
	{
		printf("unlock semop() failed\n");
		logger("ERROR", "(UNLOCK) in semop() %s", strerror(errno));

		arg.val = 0;

		if ((semctl(sem_id, 0, SETVAL, arg)) == -1)
		{
			perror("semctl()");
			logger("ERROR", "in semctl() %s", strerror(errno));
			return -2;
		}

		return -1;
	}

	/*if ((semctl(sem_id, 0, IPC_RMID)) == -1)
	{
		printf("unlock semctl() failed\n");
		logger("ERROR", "(UNLOCK) in semctl() %s", strerror(errno));
		return -1;
	}*/

	return 0;
}

int create_semaphore (int dev_ind)
{
	int semid;
	key_t sem_key;
	int semflg = 0666;
	int nsems = 1;
	union semun arg;
	char * sem_fn = (char*)"/tmp/i2c_semaphore_1.txt";

	FILE* sem_fd = fopen (sem_fn, "wr"); // create a file for semaphore
	if (sem_fd == NULL)
	{
		printf ("cannot open semaphore file\n");
		logger("ERROR", "in fopen() %s", strerror(errno));
		return -1;
	}

    	/* generate key */
    	if ((sem_key = ftok(sem_fn, (char)dev_ind)) == -1)
	{
        	printf("ftok failed, dev_ind: %d\n", dev_ind);
		logger("ERROR", "in ftok() %s", strerror(errno));
		return -2;
	}

    	/* set up semaphore */
    	if ((semid = semget(sem_key, nsems, semflg)) == -1)
	{
        	if (errno == ENOENT)
		{
			if ((semid = semget(sem_key, nsems, 0666 | IPC_CREAT | IPC_EXCL)) == -1)
			{
				perror("(repeat) semget()");
				logger("ERROR","(repeat) in semget() %s", strerror(errno));
				return -3;
			}
		}
		else
		{
			perror("semget()");
			return -3;
		}

	}

    	fd_semid[dev_ind] = semid; // store semaphore ID in map

	arg.val = 0;

	if ((semctl(semid, 0, SETVAL, arg)) == -1)
	{
		perror("semctl()");
		logger("ERROR", "in semctl() %s", strerror(errno));
		return -4;
	}

	return 0;
}

int reset_semaphore (int dev_ind)
{
	int semid = fd_semid[dev_ind];
	union semun arg;

	arg.val = 0;

	if ((semctl(semid, 0, SETVAL, arg)) == -1)
	{
		perror("semctl()");
		return -1;
	}


	return 0;
}

void lock (int i2c_bus)
{
	if (sigemptyset (&x) < 0)
	{
		logger("ERROR", "in sigemptyset() %s", strerror(errno));
	}

	if (sigaddset(&x, SIGUSR1) < 0)
	{
		logger("ERROR", "in sigaddset() %s", strerror(errno));
	}

	if (sigprocmask(SIG_BLOCK, &x, NULL) < 0)
	{
		logger("ERROR", "in sigprocmask() (SIG_BLOCK) %s", strerror(errno));
	}

	if (lock_device (i2c_bus) < 0)
	{
		logger("ERROR", "lock_device() failed for sensor bus %d", i2c_bus);
	}
}

void unlock (int i2c_bus)
{
	if (unlock_device (i2c_bus) < 0)
	{
		logger("ERROR", "unlock_device() failed for sensor bus %d", i2c_bus);
	}

	if (sigprocmask(SIG_UNBLOCK, &x, NULL) < 0)
	{
		logger("ERROR", "in sigprocmask() (SIG_UNBLOCK) %s", strerror(errno));
	}
}
