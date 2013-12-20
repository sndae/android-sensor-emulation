/*
 *   Copyright (C) 2013  Raghavan Santhanam, raghavanil4m@gmail.com, rs3294@columbia.edu
 *   This was done as part of my MS thesis research at Columbia University, NYC in Fall 2013.
 *
 *   ForOrientationSensor.cpp is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ForOrientationSensor.cpp is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>
#include <sys/types.h>

#include <hardware/sensors.h>

/************************** Orientation Sensor Emulation *************************/
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hardware/sensors.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#include <poll.h>

#include <pthread.h>

#define ONLY_READING

#ifdef ONLY_READING

static FILE *readings_fp;
#define INIT_LOG_READING do {\
				if (!readings_fp) {\
					readings_fp = fopen("/data/orient_readings", "w");\
				}\
			} while(0)
#define LOG_READING do {\
			if (readings_fp && readings[0]) {\
				struct timespec t = { 0 };\
				clock_gettime(CLOCK_REALTIME, &t);\
				unsigned long long int ts = t.tv_sec * 1E9 + t.tv_nsec;\
				fprintf(readings_fp, "[Orientation] %lluns : %s\n", ts, readings);\
				fflush(readings_fp);\
			}\
		} while(0)

#else

#define INIT_LOG_READING
#define LOG_READING

#endif

#define ONLY_ERROR
static FILE *fp;

#ifdef ONLY_ERROR

#define INITIALIZE_ERR_LOG do {\
				if (!fp) {\
					fp = fopen("/data/orient_sensor_log", "w");\
				}\
			} while(0)
#define ERR(...) (void)(fp && fprintf(fp, "%s %d: ERROR - ", __func__, __LINE__) \
								&& fprintf(fp, __VA_ARGS__) && fflush(fp))
#define ERR_SERVER(...) (void)(fp && fprintf(fp, "%s %d: ERROR - ", __func__, __LINE__) \
								&& fprintf(fp, __VA_ARGS__) && fflush(fp))
#else

#define INITIALIZE_ERR_LOG
#define ERR(...)

#endif

// #define LOG_HIGH

#ifdef LOG_HIGH

#define LOG_SERVER_HIGH(...) (void)(fp && fprintf(fp, "%s %d: ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))

#else

#define LOG_SERVER_HIGH(...)

#endif

#define LOG_BASIC

#ifdef LOG_BASIC

#define INITIALIZE_LOG do {\
				if (!fp) {\
					fp = fopen("/data/orient_sensor_log", "w");\
				}\
			} while(0)
#define LOG(...) (void)(fp && fprintf(fp, "%s %d: ", __func__, __LINE__) && fprintf(fp, __VA_ARGS__) && fflush(fp))
#define LOG_SERVER(...) (void)(LOG(__VA_ARGS__))
#define LOG_LINE LOG(" ")

#else

#define INITIALIZE_LOG
#define LOG(...)
#define LOG_SERVER(...)
#define LOG_LINE

#endif

#define READINGS_BUF_SIZE 100 /* 3 Readings */

#define ORIENTATION_SERVER_PORT 5005
#define MAX_TOLERANCE_FOR_SAME_READINGS 4

static void cleanup(void);

static void sigsegv_handler(int sig)
{
	LOG("** ATTENTION ** SIGSEGV(%d) raised!\n", sig);
	cleanup();
	exit(0);
}

static void sigabrt_handler(int sig)
{
	LOG("** ATTENTION ** SIGABRT(%d) raised!\n", sig);
	cleanup();
	exit(0);
}

static bool initialized;

static int listenfd = -1;
static int connfd = -1;
static bool connected;
static sensors_event_t sensor_data;

static pthread_t orient_readings_server_th_id = -1;

static void cleanup_orient_server(void)
{
	LOG("Cleaning up . . .\n");

	LOG("Closing listeners . . .\n");
	if (listenfd != -1) {
		bool closed = close(listenfd) != -1;
		if (!closed) {
			ERR("close - %s\n", strerror(errno));
		} else {
			LOG("Closed!\n");
		}
		listenfd = -1;
	}
	LOG("Closing connections . . .\n");
	if (connfd != -1) {
		bool closed = close(connfd) != -1;
		if (!closed) {
			ERR("close - %s\n", strerror(errno));
		} else {
			LOG("Closed!\n");
		}
		connfd = -1;
	}
	connected = false;
}

static void cleanup(void)
{
	LOG("Cleaning . . .\n");
	cleanup_orient_server();
	LOG("Cleaned!\n");
}

struct orient_server_data {
	int sensor_id;
	int port;
};

static void *orient_readings_server(void *arg)
{
	struct orient_server_data *data = reinterpret_cast<struct orient_server_data *>(arg);
	int id = data->sensor_id;
	int port = data->port;
	struct timespec t = { .tv_sec = 0, .tv_nsec = 100ULL, };
	int yes = 0;
	bool socket_opt_set = false;
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	bool bound = false;
	bool listening = false;

	LOG_SERVER("\n\n** Emulator server for Orientation - Started! **\n");
	LOG_SERVER("Data : %p\n", (void *)data);
	LOG_SERVER("Sensor id : %d\n", id);
	LOG_SERVER("Port : %d\n\n", port);

	free(data);
	data = NULL;

	LOG_SERVER("Opening socket . . .\n");
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	if (listenfd == -1) {
		ERR_SERVER("socket - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Opened!\n");

	LOG_SERVER("Setting options . . .\n");
	yes = 1;
	socket_opt_set = setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) != -1;
	if (!socket_opt_set) {
		ERR_SERVER("setsockopt - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Set!\n");

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port);

	LOG_SERVER("Binding socket . . .\n");
	bound = bind(listenfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) != -1;
	if (!bound) {
		ERR_SERVER("bind - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Bound!\n");

	LOG_SERVER("Trying to listen . . .\n");
	listening = listen(listenfd, 10) != -1;
	if (!listening) {
		ERR_SERVER("listen - %s\n", strerror(errno));
		goto done;
	}
	LOG_SERVER("Listening!\n");

	connfd = -1;

	while (1) {
		connected = false;

		LOG_SERVER("Waiting to accept . . .\n");
		connfd = accept(listenfd, (struct sockaddr *)NULL, NULL);
		if (connfd == -1) {
			ERR_SERVER("accept - %s\n", strerror(errno));
			goto done;
		}
		LOG_SERVER("Accepted!\n");

		char last_readings[READINGS_BUF_SIZE + 1] = "";
		int same_r = 0;

		while (1) {
			char readings[READINGS_BUF_SIZE + 1] = "";

			LOG_SERVER_HIGH("Receiving . . .\n");
			ssize_t bytes_received = recvfrom(connfd, readings, READINGS_BUF_SIZE + 1, MSG_WAITALL, NULL, 0);
			LOG_READING; // Log immediately.

			if (bytes_received == -1) {
				ERR_SERVER("recvfrom - %s\n", strerror(errno));
				break;
			}
			LOG_SERVER_HIGH("Received %lu bytes!\n", bytes_received);
			LOG_SERVER_HIGH("Readings: %s\n", readings);

			bool same = !strcmp(readings, last_readings);
			if (same) {
				same_r++;
				if (same_r == MAX_TOLERANCE_FOR_SAME_READINGS) {
					LOG_SERVER("Same readings %d times. Likely a problem! Resetting connection . . .\n", same_r);
					ERR_SERVER("Same readings %d times. Likely a problem! Resetting connection . . .\n", same_r);
					break;
				}
			}
			strcpy(last_readings, readings);

			connected = true;

			bool device_locked = !readings[0];
			if (device_locked) {
				LOG_SERVER("Device is likely in locked state!\n");
				continue;
			}

			if (bytes_received == 0) {
				LOG_SERVER("Zero bytes received! Likely a faulty socket. Accepting again.\n");
				break;
			}

			sensor_data.sensor = id;

			LOG_SERVER_HIGH("Sensor: Orientation\n");
			char *f_d = strchr(readings, '|');
			*f_d = '\0';
			char *s_d = strchr(f_d + 1, '|');
			*s_d = '\0';
			char *t_d = strchr(s_d + 1, '|');
			*t_d = '\0';

			sscanf(readings, "%f", &sensor_data.orientation.azimuth);
			sscanf(f_d + 1, "%f", &sensor_data.orientation.pitch);
			sscanf(s_d + 1, "%f", &sensor_data.orientation.roll);
			int d = 0;
			sscanf(t_d + 1, "%d", &d); // 8-bit data read as 32-bit as
							// there is no 8-bit integer
							// format specifier!
			sensor_data.orientation.status = d; // No data loos
								// as it's indeed
								// 8-bit data.
			sensor_data.type = SENSOR_TYPE_ORIENTATION;

			nanosleep(&t, NULL);
		}

		close(connfd);
		connfd = -1;
			
		nanosleep(&t, NULL);
	}
	
done:
	cleanup_orient_server();
	LOG_SERVER("** Orientation server - Terminated! **\n");

	return NULL;
}

static bool create_orient_server_threads(void)
{
	static bool fine = true;

	int ret = 0;
	bool created = false;

	LOG("Creating orient server . . .\n");

	LOG("Creating thread . . .\n");
	pthread_t id = -1;
	struct orient_server_data *d = reinterpret_cast<struct orient_server_data *>(malloc(sizeof(*d)));
	if (!d) {
		ERR("malloc - %s\n", strerror(errno));
		goto done;
	}

	memset(d, 0, sizeof(*d));
	d->sensor_id = '_ypr';
	d->port =  ORIENTATION_SERVER_PORT;

	ret = pthread_create(&id, NULL, orient_readings_server, d); // No way of stopping
				// this thread once started, as far as I know!
				// So, there is no such function which cleans up this
				// created thread!

	created = ret == 0;
	if (!created) {
		fine = false;
		errno = ret;
		ERR("pthread_create - Thread *failed* to create - %s\n", strerror(errno));
	} else {
		LOG("Thread created!\\m/\n");
		orient_readings_server_th_id = id;
	}
	LOG("Created!\n");

	LOG("Created!\n");

done:

	return fine;
}

static bool initialize_orient_readings_server(void)
{
	bool success = create_orient_server_threads();

	return success;
}

/************************************************************************************/

// To be part of OrientationSensor() -- This is the entire function.
	(void)signal(SIGSEGV, sigsegv_handler);
	(void)signal(SIGABRT, sigabrt_handler);

	INITIALIZE_LOG;
	INITIALIZE_ERR_LOG;
	INIT_LOG_READING;

	if (!initialized) {
		LOG("Not initialized before. Trying to initialize . . .\n");
		cleanup();
		initialized = initialize_orient_readings_server();
		if (!initialized) {
			LOG("Orient server failed to initialize.\n");
		} else {
			LOG("Orient server initialized!\n");
		}
	} else {
		LOG("Orient server already initialized!\n");
	}
// To be part of OrientationSensor() -- This is the entire function.

// To be part of process() -- This is the entire function.
	*outEvent = event;
	*outEvent = sensor_data;

	bool processed = connected;

	connected = false;

	struct timespec t;
	memset(&t, 0, sizeof(t));
	bool timed = clock_gettime(CLOCK_MONOTONIC, &t) != -1;
	if (!timed) {
		ERR("clock_gettime - %s\n", strerror(errno));
		goto done;
	}	

	{	
		int64_t ts = (int64_t)t.tv_sec * 1000000000ULL + (int64_t)t.tv_nsec;

		outEvent->timestamp = ts;
	}

done:
	return processed;
// To be part of process() -- This is the entire function.

// To be part of getSensor().
    // Sensor name is "Orientation Sensor Emulation!!!!"
    // Sensor vendor is "Columbia University - NYC - Thesis - Raghavan Santhanam"
    // Sensor version is 1
    // Sensor handle is '_ypr'
    // Sensor type is SENSOR_TYPE_ORIENTATION
    // Sensor maxRange is 360.0f
    // Sensor resolution is 1.0f/256.0f
    // Sensor power is 13.130000f
    // Sensor minDelay is 20000
// To be part of getSensor().


