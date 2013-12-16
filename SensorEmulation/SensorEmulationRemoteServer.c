/*
 *   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 *   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 *
 *   SensorEmulationRemoteServer.c is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   SensorEmulationRemoteServer.c is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * SensorEmulationRemoteServer.c
 *
 * Author: Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 * Date : Sep 30th, 2013 - Nov 10, 2013
 *
 * Work done as part of my MS in CS thesis at Columbia University, NYC.
 *
 * Working:
 *
 * Simple socket-communication server providing any connected client
 * with randomly generated sensor readings in a pre-defined pattern.
 */

#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h> 
#include <stdbool.h>
#include <signal.h>
#include <setjmp.h>

#include <pthread.h>

#define DEBUG

#ifdef DEBUG

#define LOG(...) (void)(printf("%s %d: ", __func__, __LINE__) && printf(__VA_ARGS__) && fflush(stdout))
#define ERR(...) (void)(fprintf(stderr, "%s %d: ERROR - ", __func__, __LINE__) && fprintf(stderr, __VA_ARGS__) && fflush(stderr))

#define LOG_SERVER(...) (void)(printf("[%s] ", sensors_name[n]), LOG(__VA_ARGS__))
#define ERR_SERVER(...) (void)(fprintf(stderr, "[%s] ", sensors_name[n]), ERR(__VA_ARGS__))

#else

#define LOG(...)
#define ERR(...)

#define LOG_SERVER
#define ERR_SERVER

#endif

#define ACCEL_MAX 3
#define MAGNET_MAX 300
#define LIGHT_MAX 200
#define PROX_MAX 5
#define GYRO_MAX 10
#define ORIENT_MAX 10
#define CORRECTED_GYRO_MAX 20
#define GRAVITY_MAX 10
#define LINEAR_ACCEL_MAX 10
#define ROTATION_VECTOR_MAX 20

#define EARTH_GRAVITY 9.80665
#define SOME_CONSTANT_FACTOR EARTH_GRAVITY

#define NUM_SENSORS 10
#define READINGS_BUF_SIZE (100) /* 3 readings. */
#define ACCEL_READINGS_BUF_SIZE (50) /* 3 readings. */
#define GYRO_READINGS_BUF_SIZE (50) /* 3 readings. */

#define BASE_PORT 5010
#define SERVER_PORT(num) (BASE_PORT + num)

enum sensors { EAccel = 0, EMagnetic = 1, ELight = 2, EProximity = 3, EGyro = 4, EOrient = 5,
		ECorrectedGyro = 6, EGravity = 7, ELinearAccel = 8, ERotationVector = 9, };
static const char *sensors_name[NUM_SENSORS] = {
							"Accelerometer",
							"Magnetic",
							"Light",
							"Proximity",
							"Gyroscope",
							"Orientation",
							"CorrectedGyroscope",
							"Gravity",
							"LinearAcceleration",
							"RotationVector",
						};


static void cleanup(void);
int connfd[NUM_SENSORS];
int listenfd[NUM_SENSORS];

static void ctrlc_handler(int sig)
{
	LOG("** Interrupted - Exiting! **\n");
	cleanup();
	LOG("** SensorEmulation Remote Server - Exited! **\n");

	putchar('\n');
	exit(0);
}

struct setjmp_data {
	pthread_t tid;
	jmp_buf sanity;
};
struct setjmp_data setjmp_d[NUM_SENSORS];

static void cleanup(void)
{
	LOG("Cleaning up . . .\n");

	int i = 0;
	while (i < NUM_SENSORS) {
		if (listenfd[i] != -1) {
			close(listenfd[i]);
			listenfd[i] = -1;
		}
		i++;
	}

	i = 0;
	while (i < NUM_SENSORS) {
		if (connfd[i] != -1) {
			close(connfd[i]);
			connfd[i] = -1;
		}
		i++;
	}

	i = 0;
	while (i < NUM_SENSORS) {
		if (setjmp_d[i].tid != -1) {
			pthread_cancel(setjmp_d[i].tid);
			setjmp_d[i].tid = -1;
		}
		i++;
	}

	LOG("Cleaned!\n");

	LOG("** SensorEmulation Remote Server - Exited! **\n");

	exit(0);
}

void init_servers_data(void)
{
	LOG("Initializing servers data . . .\n");

	int i = 0;
	while (i < NUM_SENSORS) {
		setjmp_d[i].tid = listenfd[i] = connfd[i] = -1;
		i++;
	}

	LOG("Initialized!\n");
}

static void sigpipe_handler(int sig)
{
	ERR("** SIGPIPE!! **\n");

	int i = 0;
	pthread_t cur_th_id = pthread_self();
	while (i < NUM_SENSORS) {
		if (setjmp_d[i].tid == cur_th_id) {
			longjmp(setjmp_d[i].sanity, 1); // Code after this point is unreachable, anyway!
			break;
		}
		i++;
	}
}

struct server_data {
	int num;
};

static void *sensor_emulation_remote_server(void *arg)
{
	(void)signal(SIGINT, ctrlc_handler);
	(void)signal(SIGPIPE, sigpipe_handler);

	struct server_data *d = arg;
	int n = d->num;
	free(d);
	d = NULL;

	int port = SERVER_PORT(n);

	setjmp_d[n].tid = pthread_self();

	while (1) {
		if (!setjmp(setjmp_d[n].sanity)) {
			LOG_SERVER("Setting up for a longjmp for any possible SIGPIPE!\n");
		} else {
			LOG_SERVER("Restoring from a SIGPIPE!\n");
			(void)signal(SIGPIPE, sigpipe_handler);
		}

		if (listenfd[n] != -1) {
			LOG_SERVER("Closing listenfd . . .\n");
			close(listenfd[n]);
			LOG_SERVER("Closed!\n");
			listenfd[n] = -1;
		}

		LOG_SERVER("Opening socket . . .\n");
 		listenfd[n] = socket(AF_INET, SOCK_STREAM, 0);
		if (listenfd[n] == -1) {
			ERR_SERVER("socket - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Opened!\n");

		LOG_SERVER("Setting socket options . . .\n");
		int yes = 1;
		bool socket_opt_set = setsockopt(listenfd[n], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
		if (!socket_opt_set) {
			ERR_SERVER("setsockopt - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Set!\n");

		struct sockaddr_in serv_addr = { 0, };
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		serv_addr.sin_port = htons(port);

		LOG_SERVER("Binding socket . . .\n");
		bool bound = bind(listenfd[n], (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
		if (!bound) {
			ERR_SERVER("bind - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Bound!\n");

		LOG_SERVER("Trying to listen . . .\n");
		bool listening = listen(listenfd[n], 10) != -1;
		if (!listening) {
			ERR_SERVER("listen - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Listening!\n");

		struct timespec t = { .tv_sec = 0, .tv_nsec = 10000ULL, };
		srand(time(NULL));

		while (1) {
			LOG_SERVER("Waiting to accept . . .\n");
			connfd[n] = accept(listenfd[n], (struct sockaddr *)NULL, NULL); 
			if (connfd[n] == -1) {
				ERR_SERVER("accept - %s\n", strerror(errno));
				break;
			}
			LOG_SERVER("Accepted!\n");


			size_t readings_size = n == EAccel ? ACCEL_READINGS_BUF_SIZE + 1 :
						n == EGyro ? GYRO_READINGS_BUF_SIZE + 1 :
								READINGS_BUF_SIZE + 1;
			char last_readings[readings_size];
			memset(last_readings, 0, sizeof(last_readings));

			while (1) {
				LOG_SERVER("Generating readings for %s . . .\n", sensors_name[n]);

				// Pretty simple logic for the sensors' fake readings. Customize as you need!
				bool valid = true;

				char gen_readings[readings_size];
				memset(gen_readings, 0, sizeof(gen_readings));

				switch (n) {
					case EAccel:
					{
						int sign = rand() % 2 ? 1 : -1;
						float x = rand() % ACCEL_MAX * EARTH_GRAVITY * sign;
						sign = rand() % 2 ? 1 : -1;	
						float y = rand() % ACCEL_MAX * EARTH_GRAVITY * sign;
						sign = rand() % 2 ? 1 : -1;
						float z = rand() % ACCEL_MAX * EARTH_GRAVITY * sign;
						sprintf(gen_readings, "%.9f|%.9f|%.9f", x, y, z);

						break;
					}
					case EMagnetic:
					{
						int sign = rand() % 2 ? 1 : -1;
						float x = rand() % MAGNET_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float y = rand() % MAGNET_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float z = rand() % MAGNET_MAX * SOME_CONSTANT_FACTOR * sign;
						sprintf(gen_readings, "%f|%f|%f", x, y, z);
								
						break;
					}
					case ELight:
					{
						float l = rand() % LIGHT_MAX;
						sprintf(gen_readings, "%f", l);

						break;
					}
					case EProximity:
					{
						float p = rand() % PROX_MAX;
						sprintf(gen_readings, "%f", p);

						break;
					}
					case EGyro:
					{
						int sign = rand() % 2 ? 1 : -1;
						float azimuth = rand() % GYRO_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float pitch = rand() % GYRO_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float roll = rand() % GYRO_MAX * SOME_CONSTANT_FACTOR * sign;
						sprintf(gen_readings, "%.9f|%.9f|%.9f", azimuth, pitch, roll);
						
						break;
					}
					case EOrient:
					{
						int sign = rand() % 2 ? 1 : -1;
						float azimuth = rand() % ORIENT_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float pitch = rand() % ORIENT_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float roll = rand() % ORIENT_MAX * SOME_CONSTANT_FACTOR * sign;
						int status = 3; // SENSOR_STATUS_ACCURACY_HIGH!
						sprintf(gen_readings, "%f|%f|%f|%d", azimuth, pitch, roll, status);
						
						break;
					}
					case ECorrectedGyro:
					{
						int sign = rand() % 2 ? 1 : -1;
						float azimuth = rand() % CORRECTED_GYRO_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float pitch = rand() % CORRECTED_GYRO_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float roll = rand() % CORRECTED_GYRO_MAX * SOME_CONSTANT_FACTOR * sign;
						sprintf(gen_readings, "%f|%f|%f", azimuth, pitch, roll);

						break;
					}
					case EGravity:
					{
						int sign = rand() % 2 ? 1 : -1;
						float lateral = rand() % GRAVITY_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float longitudinal = rand() % GRAVITY_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float vertical = rand() % GRAVITY_MAX * SOME_CONSTANT_FACTOR * sign;
						sprintf(gen_readings, "%f|%f|%f", lateral, longitudinal, vertical);
						
						break;
					}
					case ELinearAccel:
					{
						int sign = rand() % 2 ? 1 : -1;
						float lateral = rand() % LINEAR_ACCEL_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float longitudinal = rand() % LINEAR_ACCEL_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float vertical = rand() % LINEAR_ACCEL_MAX * SOME_CONSTANT_FACTOR * sign;
						sprintf(gen_readings, "%f|%f|%f", lateral, longitudinal, vertical);
						
						break;
					}
					case ERotationVector:
					{
						int sign = rand() % 2 ? 1 : -1;
						float d1 = rand() % ROTATION_VECTOR_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;	
						float d2 = rand() % ROTATION_VECTOR_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float d3 = rand() % ROTATION_VECTOR_MAX * SOME_CONSTANT_FACTOR * sign;
						sign = rand() % 2 ? 1 : -1;
						float d4 = rand() % ROTATION_VECTOR_MAX * SOME_CONSTANT_FACTOR * sign;
						sprintf(gen_readings, "%f|%f|%f|%f", d1, d2, d3, d4);

						break;
					}
					default:
					{
						valid = false;
						LOG_SERVER("Unknown sensor - number : %d\n", n);
						break;
					}
				}

				if (valid) {
					bool not_same = strcmp(gen_readings, last_readings);
					if (not_same) {					
						LOG_SERVER("Sending generated readings: %s\n", gen_readings);
						ssize_t bytes_wrote = write(connfd[n], gen_readings, readings_size);
						if (bytes_wrote == -1) {
							ERR_SERVER("write - %s\n", strerror(errno));
							break;
						}
						LOG_SERVER("Sent %zd bytes . . .!\n", bytes_wrote);

						strcpy(last_readings, gen_readings);
					} else {
						LOG_SERVER("Same readings! Not sending.\n");
					}
				}

				nanosleep(&t, NULL);
			}

			close(connfd[n]);
			connfd[n] = -1;
		}
	}

done:
	cleanup();
	return NULL;
}


int main(void)
{
	LOG("** SensorEmulation Remote Server - Started! **\n");

	init_servers_data();

	int i = 0;
	pthread_t tid[NUM_SENSORS];
	while (i < NUM_SENSORS) {
		struct server_data *d = malloc(sizeof(*d));
		if (d) {
			d->num = i;
			pthread_create(&tid[i], NULL, sensor_emulation_remote_server, d);
		} else {
			ERR("malloc - %s\n", strerror(errno));
		}
		i++;
	}

	i = 0;	
	while (i < NUM_SENSORS) {
		if (tid[i] != -1) {
			bool joined = !pthread_join(tid[i], NULL);
			if (!joined) {
				ERR("pthread_join for thread %d - %s\n", i, strerror(errno));
			}
		}
		i++;
	}

	LOG("** SensorEmulation Remote Server - Terminated! **\n");

	cleanup();

	return 0;
}

